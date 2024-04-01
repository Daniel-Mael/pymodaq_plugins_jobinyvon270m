from pymodaq.control_modules.move_utility_classes import DAQ_Move_base, comon_parameters_fun, main, DataActuatorType, \
    DataActuator  # common set of parameters for all actuators
from pymodaq.utils.daq_utils import ThreadCommand  # object used to send info back to the main thread
from pymodaq.utils.parameter import Parameter
from pyvisa.constants import ControlFlow, Parity, StopBits
from pymeasure.instruments.jobinyvon.spectro270m import JY270M
from pymodaq_plugins_horiba.utils import Config
import serial.tools.list_ports


config = Config()

"Find available COM ports."
ports = [str(port)[0:4] for port in list(serial.tools.list_ports.comports())]


class DAQ_Move_Jobinyvon270M(DAQ_Move_base):
    """ Instrument plugin class for an actuator.

    This object inherits all functionalities to communicate with PyMoDAQ’s DAQ_Move module through inheritance via
    DAQ_Move_base. It makes a bridge between the DAQ_Move module and the Python wrapper of a particular instrument.

    TODO Complete the docstring of your plugin with:
        * The set of controllers and actuators that should be compatible with this instrument plugin.
        * With which instrument and controller it has been tested.
        * The version of PyMoDAQ during the test.
        * The version of the operating system.
        * Installation instructions: what manufacturer’s drivers should be installed to make it run?"""

    _controller_units = 'nm'
    is_multiaxes = False
    _axis_names = []
    _epsilon = 0.03125

    data_actuator_type = DataActuatorType['DataActuator']

    params = [{'title': 'COM Port:', 'name': 'com_port', 'type': 'list', 'limits': ports,
               'value': config('com_port')},
              {'title': 'Slits:', 'name': 'slits', 'type': 'group', 'expanded': True, 'children': [
                  {'title': 'Entry slit (µm):', 'name': 'entry_slit', 'type': 'float',
                   'value': config('slits', 'entry'), 'min': 0.0, 'max': 7000.0},
                  {'title': 'Exit slit (µm):', 'name': 'exit_slit', 'type': 'float',
                   'value': config('slits', 'exit'), 'min': 0.0, 'max': 7000.0}]}] \
             + comon_parameters_fun(is_multiaxes, axis_names=_axis_names, epsilon=_epsilon)

    def ini_attributes(self):
        self.controller: JY270M = None
        "The minimal resolution value is self._epsilon."
        self.settings.child("epsilon").setLimits((self._epsilon, 1000))

    def get_actuator_value(self):
        """
        Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        while self.controller.motor_busy_check():
            pass
        pos = DataActuator(
            data=self.controller.get_grating_wavelength())
        pos = self.get_position_with_scaling(pos)
        return pos

    def commit_settings(self, param: Parameter):
        """
        Apply the consequences of a change of value in the detector settings.

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user.
        """
        if param.name() == "entry_slit":
            "So that the slit has time to settle."
            self.controller.timeout = 10000
            self.controller.move_entry_slit_microns(param.value())
            self.controller.timeout = self.controller.default_timeout
        elif param.name() == "exit_slit":
            "So that the slit has time to settle."
            self.controller.timeout = 10000
            self.controller.move_exit_slit_microns(param.value())
            self.controller.timeout = self.controller.default_timeout

    def ini_stage(self, controller=None):
        """
        Actuator communication initialization.

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True.
        """
        self.controller: JY270M = self.ini_stage_init(
            old_controller=controller,
            new_controller=JY270M(self.settings['com_port'],
                                  ))

        info = "Setting up Jobin Yvon 270M spectrometer and activating intelligent communication mode..."
        initialized = self.controller.auto_baud()
        if not initialized:
            raise IOError('The spectrometer could not be initialized, please reset the instrument.')

        self.controller.motor_init()
        self.get_actuator_value()
        self.controller.move_entry_slit_microns(self.settings['slits', 'entry_slit'])
        self.controller.move_exit_slit_microns(self.settings['slits', 'exit_slit'])
        return info, initialized

    def move_abs(self, value: DataActuator):
        """
        Move the actuator to the absolute target defined by value.

        Parameters
        ----------
        value: (float) value of the absolute target positioning.
        """
        value = self.check_bound(value)  # if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one
        self.controller.move_grating_wavelength(value.value())

    def move_rel(self, value: DataActuator):
        """
        Move the actuator to the relative target actuator value defined by value.

        Parameters
        ----------
        value: (float) value of the relative target positioning.
        """
        value = self.check_bound(self.current_position + value) - self.current_position
        self.target_value = value + self.current_position
        value = self.set_position_relative_with_scaling(value)
        value = value * self.controller.steps_in_one_nanometer
        self.controller.gsteps = value.value()

    def move_home(self):
        """
        Call the reference method of the controller.
        """
        self.controller.move_grating_wavelength(self.controller.lambda_0)

    def stop_motion(self):
        """
        Stop the actuator and emit the move_done signal.
        """
        self.controller.motor_stop()  # Command to stop the movement of the motor.


if __name__ == '__main__':
    main(__file__, init=False)
