# todos: add store and read registerd position from file


from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Optional

import numpy as np


class PlateHandler:
    """
    Control an xArm-based build-plate transfer system.

    The class stores every calibrated position and its current plate occupancy
    in one dictionary named ``plate_positions``. Each entry has this form::

        {
            "position_id": "Jubilee",
            "pose": np.array([x, y, z, roll, pitch, yaw]),
            "loaded_plate": "plate_1",  # or None
        }
    """

    def __init__(
        self,
        arm: Any,
        *,
        default_move_speed: float = 100,
        probing_speed: float = 5,
        probing_step: float = 0.5,
    ) -> None:
        """
        Create a PlateHandler around an existing xArm API object.

        Parameters
        ----------
        arm:
            Existing xArm API object created outside this class.
        default_move_speed:
            Default speed in mm/s for plate-transfer movements.
        probing_speed:
            Slow speed used during contact probing.
        probing_step:
            Incremental probe distance in millimeters.

        Notes
        -----
        The constructor does not move the robot. Call ``initialize()`` before
        calibration or plate-transfer operations.
        """
        if arm is None:
            raise ValueError("arm must be an initialized xArm API object.")
        if default_move_speed <= 0:
            raise ValueError("default_move_speed must be greater than zero.")
        if probing_speed <= 0:
            raise ValueError("probing_speed must be greater than zero.")
        if probing_step <= 0:
            raise ValueError("probing_step must be greater than zero.")

        self.arm = arm
        self.default_move_speed = float(default_move_speed)
        self.probing_speed = float(probing_speed)
        self.probing_step = float(probing_step)

        # positions and plate occupancy.
        self.plate_positions: dict[str, dict[str, Any]] = {}
        self._initialized = False

    # ------------------------------------------------------------------
    # Public setup and registry methods
    # ------------------------------------------------------------------

    def initialize(self) -> None:
        """
        Apply the same xArm configuration used in the original notebook.

        The method clears warnings/errors, enables motion, sets the world and
        TCP offsets, configures the unloaded tool payload, and returns the xArm
        to normal position-control mode. It does not home or reposition the arm.
        """
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.motion_enable(enable=True)
        self.arm.set_world_offset([0, 0, 8, 0, 0, 0])
        self.arm.set_tcp_offset([119.569, 0, 12.6, 0, 0, -1])
        self.arm.set_state(state=0)
        self.arm.set_tcp_load(
            weight=0.4997 + 0.048,
            center_of_gravity=[41, 0, 18],
        )
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self._initialized = True

    def register_position(
        self,
        position_id: str,
        *,
        overwrite: bool = False,
        save_to_file: bool = False,
        position_file_path: str = "./position.json",
    ) -> dict[str, Any]:
        """
        Rough-align manually, fine-align automatically, and register a position.

        1. Enter teach mode for manual rough alignment.
        2. Ask the user to align the tool corner with the position marker.
        3. Return to position-control mode.
        4. Automatically level roll/pitch using three Z probes.
        5. Automatically correct yaw using two edge probes.
        6. Search along Y for the raised reference point or edge feature.
        7. Move to the final safe reference pose.
        8. Save the calibrated pose in ``self.plate_positions``.

        Parameters
        ----------
        position_id:
            Unique position name such as ``"Jubilee"`` or ``"Shelf_1"``.
        overwrite:
            If False, an existing position cannot be recalibrated. If True, the
            pose is replaced while preserving its current ``loaded_plate``.
        save_to_file:
            If True, add or update this position in ``position_file_path`` after
            calibration.
        position_file_path:
            JSON file used when ``save_to_file`` is True. The file is created if
            it does not already exist.

        Returns
        -------
        dict
            The registered position dictionary containing ``position_id``,
            ``pose``, and ``loaded_plate``.

        Warning
        -------
        This method physically moves the robot. Keep the workspace clear and
        keep the emergency stop accessible.
        """
        self._require_initialized()
        self._validate_identifier(position_id, "position_id")

        if position_id in self.plate_positions and not overwrite:
            raise ValueError(
                f"Position '{position_id}' is already registered. "
                "Use overwrite=True to recalibrate it."
            )

        loaded_plate = None
        if position_id in self.plate_positions:
            loaded_plate = self.plate_positions[position_id]["loaded_plate"]

        self.arm.set_mode(2)
        self.arm.set_state(0)
        self.arm.start_record_trajectory()
        input(
            f"Manually align the tool corner with the marker for "
            f"'{position_id}', then press Enter."
        )
        self.arm.stop_record_trajectory()
        self.arm.set_mode(0)
        self.arm.set_state(0)

        # rough-to-fine alignment sequence
        self.arm.set_tool_position(x=-20, speed=50, wait=True)
        self.arm.set_tool_position(z=-30, speed=50, wait=True)
        self.arm.set_tool_position(x=60, speed=50, wait=True)
        self._level_position()
        self.arm.set_tool_position(x=-60, speed=50, wait=True)
        self.arm.set_tool_position(z=17, speed=50, wait=True)
        self._align_yaw()
        self._find_plate_edge()
        self.arm.set_tool_position(z=30, speed=50, wait=True)

        plate_position = {
            "position_id": position_id,
            "pose": self._get_current_pose(),
            "loaded_plate": loaded_plate,
        }
        self.plate_positions[position_id] = plate_position
        if save_to_file:
            self.save_position(position_id, position_file_path)
        return plate_position

    def save_position(
        self,
        position_id: str,
        position_file_path: str = "./position.json",
    ) -> Path:
        """
        Add or update one registered position in a JSON file.

        Parameters
        ----------
        position_id:
            Registered position to save, such as ``"Shelf_1"``.
        position_file_path:
            Destination JSON file. The file is created if needed.

        Returns
        -------
        Path
            Path of the saved JSON file.
        """
        self._validate_identifier(position_id, "position_id")
        position = self.get_position(position_id)

        path = Path(position_file_path)
        path.parent.mkdir(parents=True, exist_ok=True)

        if path.exists():
            file_text = path.read_text(encoding="utf-8")
            data = json.loads(file_text) if file_text.strip() else {}
            if not isinstance(data, dict):
                raise ValueError("Position file must contain a JSON object.")
        else:
            data = {}

        data[position_id] = self._position_to_json(position)
        path.write_text(
            json.dumps(data, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        return path

    def save_positions(self, position_file_path: str = "./position.json") -> Path:
        """
        Save all registered positions and plate occupancy to a JSON file.

        Parameters
        ----------
        position_file_path:
            Destination JSON file. Parent directories are created as needed.

        Returns
        -------
        Path
            Path of the saved JSON file.
        """
        path = Path(position_file_path)
        path.parent.mkdir(parents=True, exist_ok=True)

        data = {
            position_id: self._position_to_json(position)
            for position_id, position in self.plate_positions.items()
        }
        path.write_text(
            json.dumps(data, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        return path

    def load_positions(
        self,
        position_file_path: str = "./position.json",
        *,
        overwrite: bool = True,
    ) -> dict[str, dict[str, Any]]:
        """
        Load registered positions from a JSON file into this PlateHandler.

        Parameters
        ----------
        position_file_path:
            Source JSON file created by ``save_positions()`` or
            ``register_position(..., save_to_file=True)``.
        overwrite:
            If True, loaded positions replace existing positions with the same
            ID. If False, loading fails when a position ID already exists.

        Returns
        -------
        dict
            The current ``plate_positions`` dictionary after loading.
        """
        path = Path(position_file_path)
        if not path.exists():
            raise FileNotFoundError(f"Position file does not exist: {path}")

        raw_data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(raw_data, dict):
            raise ValueError("Position file must contain a JSON object.")

        loaded_positions = {}
        for position_id, position_data in raw_data.items():
            self._validate_identifier(position_id, "position_id")
            if position_id in self.plate_positions and not overwrite:
                raise ValueError(
                    f"Position '{position_id}' is already loaded. "
                    "Use overwrite=True to replace it."
                )
            loaded_positions[position_id] = self._position_from_json(
                position_id,
                position_data,
            )

        self.plate_positions.update(loaded_positions)
        return self.plate_positions

    def register_plate(
        self,
        plate_id: str,
        position_id: str,
        *,
        overwrite: bool = False,
    ) -> None:
        """ 
        Record that a plate is currently located at a registered position. 
        This method only updates the internal registry; it does not move the robot. 

        Parameters 
        ---------- 
        plate_id: 
            Unique plate name, such as ``"plate_1"``.

        position_id: 
            Registered position containing the plate, such as ``"Shelf_1"``. 

        overwrite: 
            If False, the plate must be unregistered and the position must be empty. If True, existing plate-location records may be replaced. 
        
        """
        self._validate_identifier(plate_id, "plate_id")
        self._validate_identifier(position_id, "position_id")

        target = self.get_position(position_id)
        current_position_id = self.get_plate_position(plate_id, required=False)

        if not overwrite:
            if target["loaded_plate"] is not None:
                raise ValueError(
                    f"Position '{position_id}' already contains plate "
                    f"'{target['loaded_plate']}'."
                )
            if current_position_id is not None:
                raise ValueError(
                    f"Plate '{plate_id}' is already registered at "
                    f"'{current_position_id}'."
                )

        if current_position_id is not None:
            self.plate_positions[current_position_id]["loaded_plate"] = None
        target["loaded_plate"] = plate_id

    def clear_position(self, position_id: str) -> None:
        """
        Mark a registered position as empty.

        Parameters
        ----------
        position_id:
            Registered position to clear, such as ``"Shelf_1"``.

        Notes
        -----
        This only updates the internal registry; it does not move the robot.
        """
        self.get_position(position_id)["loaded_plate"] = None

    def get_position(self, position_id: str) -> dict[str, Any]:
        """
        Return the stored information for a registered position.

        Parameters
        ----------
        position_id:
            Registered position to retrieve, such as ``"Jubilee"``.

        Returns
        -------
        dict
            Position ID, calibrated pose, and currently loaded plate.
        """
        self._validate_identifier(position_id, "position_id")
        try:
            return self.plate_positions[position_id]
        except KeyError as exc:
            raise KeyError(
                f"Position '{position_id}' has not been registered."
            ) from exc

    def get_plate_position(
        self,
        plate_id: str,
        *,
        required: bool = True,
    ) -> Optional[str]:
        """
        Return the registered position currently containing a plate.

        Parameters
        ----------
        plate_id:
            Plate to locate, such as ``"plate_1"``.

        required:
            If True, raise an error when the plate is not registered.
            If False, return None instead.

        Returns
        -------
        Optional[str]
            Position ID containing the plate, or None if not found.
        """

        self._validate_identifier(plate_id, "plate_id")
        for position_id, position in self.plate_positions.items():
            if position["loaded_plate"] == plate_id:
                return position_id
        if required:
            raise KeyError(
                f"Plate '{plate_id}' is not registered at any position."
            )
        return None

    def get_plate_at_position(self, position_id: str) -> Optional[str]:
        """
        Return the plate currently registered at a position.

        Parameters
        ----------
        position_id:
            Registered position to check, such as ``"Jubilee"``.

        Returns
        -------
        Optional[str]
            Plate ID at the position, or None if the position is empty.
        """
        return self.get_position(position_id)["loaded_plate"]

    def get_status(self) -> dict[str, Optional[str]]:
        """
        Return the current plate occupancy of all registered positions.

        Returns
        -------
        dict
            Mapping from each position ID to its plate ID or None.
        """
        return {
            position_id: position["loaded_plate"]
            for position_id, position in self.plate_positions.items()
        }

    # ------------------------------------------------------------------
    # Public physical transfer methods
    # ------------------------------------------------------------------

    def move_plate(
        self,
        plate_id: str,
        destination_position_id: str,
        *,
        move_speed: Optional[float] = None,
    ) -> None:
        """
        Move a plate from its current position to an empty destination.

        Parameters
        ----------
        plate_id:
            Plate to move, such as ``"plate_1"``.

        destination_position_id:
            Registered destination position id, such as ``"Jubilee"``.

        move_speed:
            Transfer speed in mm/s. If None, use ``default_move_speed``.

        Notes
        -----
        The source position is found automatically from ``plate_id``.
        """
        self._require_initialized()
        self._validate_identifier(plate_id, "plate_id")
        self._validate_identifier(
            destination_position_id,
            "destination_position_id",
        )

        source_position_id = self.get_plate_position(plate_id)
        source = self.get_position(source_position_id)
        destination = self.get_position(destination_position_id)

        if source_position_id == destination_position_id:
            return
        if destination["loaded_plate"] is not None:
            raise ValueError(
                f"Destination '{destination_position_id}' is occupied by "
                f"plate '{destination['loaded_plate']}'."
            )

        self._transfer_plate(
            source_pose=source["pose"],
            destination_pose=destination["pose"],
            move_speed=self._resolve_speed(move_speed),
        )

        source["loaded_plate"] = None
        destination["loaded_plate"] = plate_id

    def exchange_plates(
        self,
        plate_id_1: str,
        plate_id_2: str,
        *,
        buffer_position_id: Optional[str] = None,
        move_speed: Optional[float] = None,
    ) -> None:
        """
        Exchange the positions of two registered plates using an empty buffer.

        Parameters
        ----------
        plate_id_1:
            First plate to exchange.

        plate_id_2:
            Second plate to exchange.

        buffer_position_id:
            Empty position used temporarily during the exchange.
            If None, an available empty position is selected automatically.

        move_speed:
            Transfer speed in mm/s. If None, use ``default_move_speed``.
        """
        self._require_initialized()
        self._validate_identifier(plate_id_1, "plate_id_1")
        self._validate_identifier(plate_id_2, "plate_id_2")

        if plate_id_1 == plate_id_2:
            raise ValueError("The two plate IDs must be different.")

        position_id_1 = self.get_plate_position(plate_id_1)
        position_id_2 = self.get_plate_position(plate_id_2)

        if buffer_position_id is None:
            buffer_position_id = self._find_empty_position(
                excluded_position_ids={position_id_1, position_id_2}
            )
        else:
            buffer = self.get_position(buffer_position_id)
            if buffer["loaded_plate"] is not None:
                raise ValueError(
                    f"Buffer position '{buffer_position_id}' is occupied by "
                    f"plate '{buffer['loaded_plate']}'."
                )
            if buffer_position_id in {position_id_1, position_id_2}:
                raise ValueError(
                    "The buffer position must differ from both source positions."
                )

        self.move_plate(
            plate_id_1,
            buffer_position_id,
            move_speed=move_speed,
        )
        self.move_plate(
            plate_id_2,
            position_id_1,
            move_speed=move_speed,
        )
        self.move_plate(
            plate_id_1,
            position_id_2,
            move_speed=move_speed,
        )

    # ------------------------------------------------------------------
    # Internal calibration methods
    # ------------------------------------------------------------------

    def _probe_z_absolute(
        self,
        sample_count: int,
    ) -> tuple[float, float, np.ndarray]:
        """
        Probe GPIO input 1 and return absolute Z-coordinate readings.

        The tool advances in small positive-Z steps until contact, records the
        current Z coordinate, then retracts 5 mm.
        """
        self._validate_sample_count(sample_count)
        readings = np.zeros(sample_count, dtype=float)
        for i in range(sample_count):
            while True:
                if self.arm.get_tgpio_digital(ionum=1)[1] == 1:
                    readings[i] = self._get_current_pose()[2]
                    self.arm.set_tool_position(z=-5, speed=50, wait=True)
                    break
                self.arm.set_tool_position(
                    z=self.probing_step,
                    speed=self.probing_speed,
                    wait=True,
                )
        return float(np.mean(readings)), float(np.std(readings)), readings

    def _probe_z_relative(
        self,
        sample_count: int,
    ) -> tuple[float, float, np.ndarray]:
        """
        Probe GPIO input 1 and return relative Z travel distances.

        After contact, the tool retracts by the measured travel distance.
        """
        self._validate_sample_count(sample_count)
        readings = np.zeros(sample_count, dtype=float)
        for i in range(sample_count):
            traveled_distance = 0.0
            while True:
                if self.arm.get_tgpio_digital(ionum=1)[1] == 1:
                    readings[i] = traveled_distance
                    self.arm.set_tool_position(
                        z=-traveled_distance,
                        speed=50,
                        wait=True,
                    )
                    break
                self.arm.set_tool_position(
                    z=self.probing_step,
                    speed=self.probing_speed,
                    wait=True,
                )
                traveled_distance += self.probing_step
        return float(np.mean(readings)), float(np.std(readings)), readings

    def _probe_x_absolute(
        self,
        sample_count: int,
    ) -> tuple[float, float, np.ndarray]:
        """
        Probe GPIO input 0 and return absolute coordinate readings.

        The recorded pose component remains index 1 to preserve the original
        notebook behavior exactly.
        """
        self._validate_sample_count(sample_count)
        readings = np.zeros(sample_count, dtype=float)
        for i in range(sample_count):
            while True:
                if self.arm.get_tgpio_digital(ionum=0)[1] == 1:
                    readings[i] = self._get_current_pose()[1]
                    self.arm.set_tool_position(x=-5, speed=50, wait=True)
                    break
                self.arm.set_tool_position(
                    x=self.probing_step,
                    speed=self.probing_speed,
                    wait=True,
                )
        return float(np.mean(readings)), float(np.std(readings)), readings

    def _probe_x_relative(
        self,
        sample_count: int,
    ) -> tuple[float, float, np.ndarray]:
        """
        Probe GPIO input 0 and return relative local-X travel distances.
        """
        self._validate_sample_count(sample_count)
        readings = np.zeros(sample_count, dtype=float)
        for i in range(sample_count):
            traveled_distance = 0.0
            while True:
                if self.arm.get_tgpio_digital(ionum=0)[1] == 1:
                    readings[i] = traveled_distance
                    self.arm.set_tool_position(
                        x=-traveled_distance,
                        speed=50,
                        wait=True,
                    )
                    break
                self.arm.set_tool_position(
                    x=self.probing_step,
                    speed=self.probing_speed,
                    wait=True,
                )
                traveled_distance += self.probing_step
        return float(np.mean(readings)), float(np.std(readings)), readings

    def _align_yaw(self) -> tuple[float, float]:
        """
        Align yaw in two stages.

        1. Use a short 10 mm probe span for an initial yaw correction.
        2. Use a longer 160 mm probe span for a more precise correction.

        Returns
        -------
        tuple[float, float]
            The initial and fine yaw corrections, in degrees.
        """
        self._probe_x_absolute(1)

        # --------------------------------------------------------------
        # Initial yaw correction using a short 10 mm probe span
        # --------------------------------------------------------------
        self.arm.set_tool_position(
            y=5,
            speed=50,
            wait=True,
        )
        first_short_distance = self._probe_x_relative(1)[0]

        self.arm.set_tool_position(
            y=-10,
            speed=50,
            wait=True,
        )
        second_short_distance = self._probe_x_relative(1)[0]

        initial_yaw_correction = -np.degrees(
            np.arctan(
                (first_short_distance - second_short_distance) / 10.0
            )
        )

        # Return to the center and retract slightly before rotating.
        self.arm.set_tool_position(
            x=-5,
            y=5,
            speed=50,
            wait=True,
        )
        self.arm.set_tool_position(
            yaw=float(initial_yaw_correction),
            speed=50,
            wait=True,
        )

        # --------------------------------------------------------------
        # Fine yaw correction using a longer 160 mm probe span
        # --------------------------------------------------------------
        self.arm.set_tool_position(
            y=80,
            speed=100,
            wait=True,
        )
        first_long_distance = self._probe_x_relative(1)[0]

        self.arm.set_tool_position(
            y=-160,
            speed=100,
            wait=True,
        )
        second_long_distance = self._probe_x_relative(1)[0]

        fine_yaw_correction = -np.degrees(
            np.arctan(
                (first_long_distance - second_long_distance) / 160.0
            )
        )

        # Return to the center and retract before applying the fine correction.
        self.arm.set_tool_position(
            x=-10,
            y=80,
            speed=100,
            wait=True,
        )
        self.arm.set_tool_position(
            yaw=float(fine_yaw_correction),
            speed=50,
            wait=True,
        )

        print(
            f"Initial yaw correction: {initial_yaw_correction:.3f}° | "
            f"Fine yaw correction: {fine_yaw_correction:.3f}°"
        )

        return (
            float(initial_yaw_correction),
            float(fine_yaw_correction),
        )
    def _find_reference_point(
        self,
        *,
        y_step: float = 0.5,
        detection_threshold: float = 0.8,
        maximum_y_travel: float = 50,
        final_y_offset: float = -40,
    ) -> float:
        """
        THIS METHOD IS OUTDATED, REPLACED WITH _find_plate_edge

        Search in positive Y until consecutive X probes differ sufficiently.

        The detected change identifies the raised point or edge feature. After
        detection, the tool moves by ``final_y_offset`` in Y.
        """
        if y_step <= 0 or detection_threshold <= 0 or maximum_y_travel <= 0:
            raise ValueError(
                "y_step, detection_threshold, and maximum_y_travel must be positive."
            )

        self._probe_x_absolute(1)
        previous_distance = self._probe_x_relative(1)[0]
        traveled_y = 0.0

        while traveled_y < maximum_y_travel:
            self.arm.set_tool_position(y=y_step, speed=50, wait=True)
            traveled_y += y_step
            current_distance = self._probe_x_relative(1)[0]
            difference = previous_distance - current_distance

            print(
                f"Y travel: {traveled_y:.2f} mm | "
                f"previous X probe: {previous_distance:.2f} mm | "
                f"current X probe: {current_distance:.2f} mm | "
                f"difference: {difference:.2f} mm"
            )

            if abs(difference) > detection_threshold:
                print("Reference point or edge feature detected.")
                self.arm.set_tool_position(
                    y=final_y_offset,
                    speed=50,
                    wait=True,
                )
                return traveled_y

            previous_distance = current_distance

        raise RuntimeError(
            f"Reference point was not detected within {maximum_y_travel} mm."
        )

    def _find_plate_edge(
        self,
        *,
        coarse_step: float = 5.0,
        fine_step: float = 0.5,
        maximum_y_travel: float = 350,
        half_plate_length: float = 145,
        coarse_speed: float = 50,
        fine_speed: float = 10,
        center_move_speed: float = 50,
    ) -> float:
        """
        Find the sloped plate edge with a two-stage search.

        A coarse search moves quickly in positive Y using larger steps. After the
        end stop is triggered, the tool moves backward to release the switch and
        performs a slower fine search using smaller steps.

        After the edge is located, the tool moves in negative Y by half the plate
        length. This places the tool approximately at the plate's longitudinal
        center.
        """
        if coarse_step <= 0 or fine_step <= 0:
            raise ValueError("Search steps must be positive.")

        if maximum_y_travel <= 0:
            raise ValueError("maximum_y_travel must be positive.")

        if half_plate_length <= 0:
            raise ValueError("half_plate_length must be positive.")

        if coarse_speed <= 0 or fine_speed <= 0 or center_move_speed <= 0:
            raise ValueError("Movement speeds must be positive.")

        # Establish the plate's side position using GPIO input 0.
        self._probe_x_absolute(1)

        # Raise the end stop slightly to avoid collision with the flat plate edge.
        self.arm.set_tool_position(
            z=-3,
            speed=10,
            wait=True,
        )

        traveled_y = 0.0

        # --------------------------------------------------------------
        # Coarse search: move quickly in larger Y increments.
        # --------------------------------------------------------------
        while traveled_y < maximum_y_travel:
            self.arm.set_tool_position(
                y=coarse_step,
                speed=coarse_speed,
                wait=True,
            )
            traveled_y += coarse_step

            if self.arm.get_tgpio_digital(ionum=0)[1] == 1:
                print(
                    f"Coarse edge detection after "
                    f"{traveled_y:.2f} mm of positive-Y travel."
                )
                break
        else:
            # Restore the end stop height before raising an error.
            self.arm.set_tool_position(
                z=3,
                speed=10,
                wait=True,
            )

            raise RuntimeError(
                f"Plate edge was not detected within "
                f"{maximum_y_travel:.2f} mm."
            )

        # --------------------------------------------------------------
        # Release the switch and move behind the detected edge.
        # Moving back two coarse steps provides space for a fine search.
        # --------------------------------------------------------------
        fine_search_start_offset = 2 * coarse_step

        self.arm.set_tool_position(
            y=-fine_search_start_offset,
            speed=coarse_speed,
            wait=True,
        )
        traveled_y -= fine_search_start_offset

        # Verify that the switch has been released.
        if self.arm.get_tgpio_digital(ionum=0)[1] == 1:
            raise RuntimeError(
                "The end stop remained triggered after the coarse-search "
                "retraction. Increase fine_search_start_offset."
            )

        # --------------------------------------------------------------
        # Fine search: locate the edge using small, slow increments.
        # --------------------------------------------------------------
        while True:
            self.arm.set_tool_position(
                y=fine_step,
                speed=fine_speed,
                wait=True,
            )
            traveled_y += fine_step

            if self.arm.get_tgpio_digital(ionum=0)[1] == 1:
                print(
                    f"Plate edge precisely detected after "
                    f"{traveled_y:.2f} mm of positive-Y travel."
                )
                break

        # The detected point is one end of the plate. Moving negative Y by
        # half the plate length places the tool at the plate's midpoint.
        self.arm.set_tool_position(
            y=-half_plate_length,
            speed=center_move_speed,
            wait=True,
        )

        # Restore the end stop to its original height.
        self.arm.set_tool_position(
            z=3,
            speed=10,
            wait=True,
        )

        print(
            f"Moved {half_plate_length:.2f} mm in negative Y "
            "from the detected edge to the plate center."
        )

        return traveled_y
    def _level_position(
        self,
    ) -> tuple[float, float, np.ndarray, np.ndarray, np.ndarray]:
        """
        Probe three points, estimate the local plane, and correct roll/pitch.

        The three local points are (0, 0), (60, 40), and (60, -40). Their Z
        values define a plane whose normal is converted into angular corrections.
        """
        x_offset = 60.0
        y_offset = 40.0
        point_1 = np.array([0.0, 0.0, 0.0])
        point_2 = np.array([x_offset, y_offset, 0.0])
        point_3 = np.array([x_offset, -y_offset, 0.0])

        self._probe_z_absolute(1)
        self.arm.set_tool_position(z=-5, speed=50, wait=True)
        point_1[2] = self._probe_z_relative(1)[0]
        self.arm.set_tool_position(
            x=x_offset,
            y=y_offset,
            speed=50,
            wait=True,
        )
        point_2[2] = self._probe_z_relative(1)[0]
        self.arm.set_tool_position(
            y=-2 * y_offset,
            speed=50,
            wait=True,
        )
        point_3[2] = self._probe_z_relative(1)[0]
        self.arm.set_tool_position(
            x=-x_offset,
            y=y_offset,
            speed=50,
            wait=True,
        )

        normal_vector = np.cross(point_2 - point_1, point_3 - point_1)
        magnitude = np.linalg.norm(normal_vector)
        if magnitude == 0:
            raise RuntimeError("The three probe points do not define a plane.")

        unit_normal = normal_vector / magnitude
        angle_components = np.degrees(np.arcsin(unit_normal))
        roll_correction = float(angle_components[1])
        pitch_component = float(angle_components[0])

        self.arm.set_tool_position(
            roll=roll_correction,
            pitch=-pitch_component,
            speed=50,
            wait=True,
        )
        return (
            roll_correction,
            pitch_component,
            point_1,
            point_2,
            point_3,
        )

    # ------------------------------------------------------------------
    # Internal physical transfer methods
    # ------------------------------------------------------------------

    def _pick_up_plate(self, source_pose: np.ndarray) -> None:
        """
        Execute the original pickup motion at a calibrated source pose.

        The method moves into the plate, applies the original +/-6 degree tilt
        sequence, updates the TCP payload to include the plate, and lifts it.
        """
        source_pose = self._validate_pose(source_pose)
        self.arm.set_position(*source_pose, speed=100, wait=True)
        self.arm.set_tool_position(x=40, speed=100, wait=True)
        self.arm.set_tool_position(pitch=-6, speed=100, wait=True)
        self.arm.set_tool_position(
            x=42,
            z=-42 * np.tan(np.radians(6)),
            speed=100,
            wait=True,
        )
        self.arm.set_tool_position(z=-2, speed=50, wait=True)
        self.arm.set_tool_position(pitch=6, speed=50, wait=True)
        self.arm.set_tcp_load(
            weight=0.4997 + 0.048 + 1.609,
            center_of_gravity=[174, 0, 11],
        )
        self.arm.set_tool_position(z=-20, speed=10, wait=True)

    def _set_down_plate(self, destination_pose: np.ndarray) -> None:
        """
        Execute the original set-down motion at a calibrated destination pose.

        The method approaches from above, inserts the plate, restores the
        unloaded TCP payload, and retracts the tool.
        """
        destination_pose = self._validate_pose(destination_pose)
        target_pose = destination_pose + np.array(
            [0, 0, 20 + 2 * np.cos(np.radians(6)), 0, 0, 0],
            dtype=float,
        )

        self.arm.set_position(*target_pose, speed=100, wait=True)
        self.arm.set_tool_position(
            x=(
                40
                + 45 / np.cos(np.radians(6))
                + 2 * np.tan(np.radians(6))
            ),
            speed=100,
            wait=True,
        )
        self.arm.set_tool_position(z=20, speed=10, wait=True)
        self.arm.set_tcp_load(
            weight=0.4997 + 0.048,
            center_of_gravity=[41, 0, 18],
        )
        self.arm.set_tool_position(pitch=-6, speed=50, wait=True)
        self.arm.set_tool_position(z=2, speed=50, wait=True)
        self.arm.set_tool_position(
            x=-42,
            z=42 * np.tan(np.radians(6)),
            speed=100,
            wait=True,
        )
        self.arm.set_tool_position(pitch=6, speed=100, wait=True)
        self.arm.set_tool_position(x=-40, speed=100, wait=True)

    def _transfer_plate(
        self,
        *,
        source_pose: np.ndarray,
        destination_pose: np.ndarray,
        move_speed: float,
    ) -> None:
        """
        Physically transfer one plate between two calibrated poses.

        This preserves the original transfer workflow: partial yaw correction,
        pickup, 450 mm retreat, two half-yaw rotations, a 350 mm destination
        approach offset, and set-down. Registry state is updated by move_plate(),
        not by this internal method.
        """
        source_pose = self._validate_pose(source_pose)
        destination_pose = self._validate_pose(destination_pose)
        current_pose = self._get_current_pose()

        self.arm.set_tool_position(
            yaw=(current_pose[5] - source_pose[5]) / 10,
            speed=move_speed,
            wait=True,
        )
        self._pick_up_plate(source_pose)
        self.arm.set_tool_position(x=-450, speed=move_speed, wait=True)

        yaw_difference = source_pose[5] - destination_pose[5]
        self.arm.set_tool_position(
            yaw=yaw_difference / 2,
            speed=move_speed,
            wait=True,
        )
        self.arm.set_tool_position(
            yaw=yaw_difference / 2,
            speed=move_speed,
            wait=True,
        )

        destination_yaw = np.radians(destination_pose[5])
        approach_pose = destination_pose + np.array(
            [
                -350 * np.cos(destination_yaw),
                -350 * np.sin(destination_yaw),
                20,
                0,
                0,
                0,
            ],
            dtype=float,
        )
        self.arm.set_position(*approach_pose, speed=move_speed, wait=True)
        self._set_down_plate(destination_pose)

    # ------------------------------------------------------------------
    # Internal validation and utility methods
    # ------------------------------------------------------------------

    def _find_empty_position(
        self,
        *,
        excluded_position_ids: Optional[set[str]] = None,
    ) -> str:
        """Return the first empty registered position not excluded."""
        excluded = excluded_position_ids or set()
        for position_id, position in self.plate_positions.items():
            if (
                position_id not in excluded
                and position["loaded_plate"] is None
            ):
                return position_id
        raise RuntimeError(
            "No empty registered position is available for use as a buffer."
        )

    def _get_current_pose(self) -> np.ndarray:
        """
        Read and validate [x, y, z, roll, pitch, yaw] from the xArm API.
        """
        result = self.arm.get_position()
        if not isinstance(result, (tuple, list)) or len(result) < 2:
            raise RuntimeError(
                "arm.get_position() returned an unexpected value."
            )
        status_code = result[0]
        if isinstance(status_code, (int, np.integer)) and status_code != 0:
            raise RuntimeError(
                f"arm.get_position() failed with status code {status_code}."
            )
        return self._validate_pose(result[1])

    def _resolve_speed(self, move_speed: Optional[float]) -> float:
        """Return a validated explicit or default movement speed."""
        speed = (
            self.default_move_speed
            if move_speed is None
            else float(move_speed)
        )
        if speed <= 0:
            raise ValueError("move_speed must be greater than zero.")
        return speed

    def _require_initialized(self) -> None:
        """Prevent physical movement before initialize() is called."""
        if not self._initialized:
            raise RuntimeError(
                "PlateHandler is not initialized. Call initialize() first."
            )

    @staticmethod
    def _validate_identifier(value: str, parameter_name: str) -> None:
        """Require position and plate identifiers to be non-empty strings."""
        if not isinstance(value, str) or not value.strip():
            raise ValueError(
                f"{parameter_name} must be a non-empty string."
            )

    @staticmethod
    def _validate_pose(pose: Any) -> np.ndarray:
        """
        Convert a pose to a float array and validate six finite components.
        """
        pose_array = np.asarray(pose, dtype=float)
        if pose_array.shape != (6,):
            raise ValueError(
                "Pose must contain [x, y, z, roll, pitch, yaw]."
            )
        if not np.all(np.isfinite(pose_array)):
            raise ValueError("Pose values must all be finite numbers.")
        return pose_array.copy()

    @staticmethod
    def _validate_sample_count(sample_count: int) -> None:
        """Require a positive integer number of probe samples."""
        if (
            not isinstance(sample_count, (int, np.integer))
            or sample_count <= 0
        ):
            raise ValueError("sample_count must be a positive integer.")

    @staticmethod
    def _position_to_json(position: dict[str, Any]) -> dict[str, Any]:
        """Convert an internal position record to JSON-serializable values."""
        return {
            "position_id": position["position_id"],
            "pose": PlateHandler._validate_pose(position["pose"]).tolist(),
            "loaded_plate": position["loaded_plate"],
        }

    @staticmethod
    def _position_from_json(
        position_id: str,
        position_data: Any,
    ) -> dict[str, Any]:
        """Convert a JSON position record to the internal representation."""
        if not isinstance(position_data, dict):
            raise ValueError(
                f"Position '{position_id}' must contain a JSON object."
            )

        file_position_id = position_data.get("position_id", position_id)
        if file_position_id != position_id:
            raise ValueError(
                f"Position key '{position_id}' does not match "
                f"position_id '{file_position_id}'."
            )

        loaded_plate = position_data.get("loaded_plate")
        if loaded_plate is not None:
            PlateHandler._validate_identifier(loaded_plate, "loaded_plate")

        return {
            "position_id": position_id,
            "pose": PlateHandler._validate_pose(position_data.get("pose")),
            "loaded_plate": loaded_plate,
        }
