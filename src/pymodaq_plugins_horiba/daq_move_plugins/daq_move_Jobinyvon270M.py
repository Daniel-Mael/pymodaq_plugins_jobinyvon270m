from pymodaq.control_modules.move_utility_classes import DAQ_Move_base, comon_parameters_fun, main, DataActuatorType, \
    DataActuator  # common set of parameters for all actuators
from pymodaq.utils.daq_utils import ThreadCommand  # object used to send info back to the main thread
from pymodaq.utils.parameter import Parameter
from pyvisa.constants import ControlFlow, Parity, StopBits
from pymodaq_plugins_horiba.hardware.horiba.spectro270m import JY270M



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
    _axis_names = []  # TODO for your plugin: complete the list
    _epsilon = 0.03125  # TODO replace this by a value that is correct depending on your controller

    data_actuator_type = DataActuatorType['DataActuator']

    params = [{'title': 'Slits:', 'name': 'slits', 'type': 'group', 'expanded': True, 'children': [
                {'title': 'Entry slit (µm):', 'name': 'entry_slit', 'type': 'float', 'value': 0.0, 'min': 0.0, 'max': 7000.0},
                {'title': 'Exit slit (µm):', 'name': 'exit_slit', 'type': 'float', 'value': 0.0, 'min': 0.0, 'max': 7000.0}]}] \
                + comon_parameters_fun(is_multiaxes, axis_names=_axis_names, epsilon=_epsilon)


    def ini_attributes(self):
        self.controller: JY270M = None

    def get_actuator_value(self):
        """Get the current value from the hardware with scaling conversion.

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

    """def close(self):
        Terminate the communication protocol
        ## TODO for your custom plugin
        self.controller.your_method_to_terminate_the_communication()  # when writing your own plugin replace this line"""

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        if param.name() == "entry_slit":
            self.controller.timeout = 10 # so that the slit has time to settle
            self.controller.move_entry_slit_microns(param.value())
        elif param.name() == "exit_slit":
            self.controller.timeout = 10 # so that the slit has time to settle
            self.controller.move_exit_slit_microns(param.value())


    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        self.controller = self.ini_stage_init(old_controller=controller,
                                              new_controller=JY270M('COM1',
                                                                    baud_rate=9600,
                                                                    timeout=300,
                                                                    parity=Parity.none,
                                                                    data_bits=8,
                                                                    stop_bits=StopBits.one,
                                                                    flow_control=ControlFlow.dtr_dsr,
                                                                    write_termination='',
                                                                    read_termination='',
                                                                    includeSCPI=False))

        info = "Setting up Jobin Yvon 270M spectrometer and activating intelligent communication mode..."
        initialized = self.controller.auto_baud()
        if not initialized:
            raise IOError('The spectrometer could not be initialized, please reset the instrument.')
        else:
            self.controller.motor_init()
            self.get_actuator_value()
        return info, initialized

    def move_abs(self, value: DataActuator):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """

        value = self.check_bound(value)  # if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one
        self.controller.move_grating_wavelength(value.value())


        #on a mis un get_actuator_value car avec le "ThreadCommand" on avait des messages d'erreurs,
        #mais le programme fonctionnait quand même
        self.get_actuator_value()
        #self.emit_status(ThreadCommand('Update_Status', [self.controller.get_grating_wavelength()]))

    def move_rel(self, value: DataActuator):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        value = self.check_bound(self.current_position + value) - self.current_position
        self.target_value = value + self.current_position
        value = self.set_position_relative_with_scaling(value)
        value = value*32
        self.controller.gsteps = value.value()
        self.emit_status(ThreadCommand('Update_Status', ['info']))

    def move_home(self):
        """Call the reference method of the controller"""

        self.controller.motor_init()
        self.emit_status(ThreadCommand('Update_Status', ['info']))

    def stop_motion(self):
        """Stop the actuator and emits move_done signal"""

        self.controller.motor_stop() # Command to stop the movement of the motor.
        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))


if __name__ == '__main__':
    main(__file__)