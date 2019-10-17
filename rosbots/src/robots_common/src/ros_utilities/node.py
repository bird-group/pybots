import rospy

class GenericNode(object):
    """Implements a generic node
    """
    def __init__(self, name, initialization_configuration='all'):
        """Constructor

        Arguments:
            name: the name of this node
            initialization_configuration: control the initialization. Specify
                the components which will be initialized. A list of components.
                Valid entries are:
                    parameters: get parameters from the parameter server
                    publishers: initialize publishers
                    subscribers: initialize subscribers
                    timers: initialize timers
                'all' can also be specified in order to initialize all of these
                components. The default behavior is 'all'

        Returns:
            class instance
        """
        super(GenericNode, self).__init__()

        rospy.init_node(name)

        self._initialization_functions = {
            'parameters': self._init_parameters,
            'publishers': self._init_publishers,
            'timers': self._init_timers,
            'subscribers': self._init_subscribers,
            }
        if initialization_configuration == 'all':
            initialization_configuration = [
                'parameters', 'publishers', 'timers', 'subscribers']
        for component in initialization_configuration:
            assert component in self._initialization_functions,\
                'component {} not valid for initialization'.format(component)
            self._initialization_functions[component]()

    def _init_parameters(self):
        """Virtual method to initialize parameters

        Arguments:
            no arguments

        Returns:
            no returns
        """
        return

    def _init_publishers(self):
        """Virtual method to initialize publishers

        Arguments:
            no arguments

        Returns:
            no returns
        """
        return

    def _init_subscribers(self):
        """Virtual method to initialize subscribers

        Arguments:
            no arguments

        Returns:
            no returns
        """
        return

    def _init_timers(self):
        """Virtual method to initialize timers

        Arguments:
            no arguments

        Returns:
            no returns
        """
        return
