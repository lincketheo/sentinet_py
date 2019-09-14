import types
import math

##
# @brief Simply a container for a python dict for now, but can have more to it
class Alphabet:
    ##
    # @brief An alphabet map is initialized by sending in a dictionary of descriptions with starting values
    #
    # @param alph_map A dictionary of string descriptions with their initialized data type
    # EX: {"Description" : Descrition_0, "X_Pos" : INFINITY, "Time" : 10 .... }
    def __init__(self, alph_map):
        self.alph_map = dict(alph_map)
    
    
    ##
    # @brief Simply add a value to the alphabet dictionary
    #
    # @param The description (a string) of the variable
    # @param value The value of the variable
    def assign_var(self, description: str, value):
        if description not in self.alph_map:
            self.invalid_description(description)
            return
        if type(self.alph_map[description]) is not type(value):
            self.invalid_type_description(description, value)
            return
        self.alph_map[description] = value

    ##
    # @brief Simply return a value from the dictionary
    #
    # @param description The value at description
    def get_var(self, description: str):
        if description not in self.alph_map:
            self.invalid_description(description)
        return self.alph_map[description]


    ##
    # @brief An error warning for an invalid description not found
    #
    # @param description The not found description
    def invalid_description(self, description):
        print("Unable to find variable %s in alphabet" % (description))


    ##
    # @brief An error warning for an invalid type
    #
    # @param description The description that had the wrong type
    # @param value The invalid type
    def invalid_type_description(self, description, value):
        print("Discrepent types, cannot assign %s to type %s" % (type(value), type(self.alph_map[description])))

""" Example:
KermitAlphabet = {"x" : math.inf, 
                  "y" : math.inf,
                  "theta" : math.inf,
                  "auger" : True,
                  "mining" : False,
                  "dumping" : False,
                  "bucket_full" : False,
                  "moving" : False,
                  "system" : False,
                  "time" : 600,
                  "kill_flag" : False}
"""



##
# @brief A State should not really be used alone, it's more of an interface. States require a heuristic object to contain them (i.e. 
# A state machine
#
# A State is simply something with a description and a list of transition functions that is able to point to another state
# You need a heuristic object because transition functions, if evaluated to true, point to a new state.
class State:
    ##
    # @brief A State itself is initialized with an empty transition function map
    #
    # @note A state has an array of transition functions that simply return a boolean
    # when evaluated. They are functions in the form:
    #
    # bool transition_function(void)
    #
    # The definition of a DSA would point to passing values from the alphabet, however,
    # this causes runtime allocation, so to pass variables to a transition function, you simply
    # record those functions within the State class, remember, this is a base class
    #
    # @param state_label The label for the state (best be an int)
    # @param state_desc The description, a python string
    def __init__(self, state_label: int, state_desc: str):
        self.state_label = state_label
        self.state_desc = state_desc
        self.transition_functions = {}
        self.alphabet = None

    ##
    # @brief Associate an alphabet with the state
    #
    # @note this is only here so that a State can pass alphabet variables to its 
    # transition functions
    #
    # @param alphabet The alphabet to associate with
    def associate_alphabet(self, alphabet: Alphabet):
        self.alphabet = alphabet

    ##
    # @brief Associate a state with a transition function
    #
    # @param func_desc The description / label of the function
    # @param state The state to associate with
    def associate_transition_function(self, func_desc, state):
        if func_desc not in self.transition_functions:
            self.invalid_transition_function(func_desc)
            return
        self.transition_functions[func_desc]["state"] = state
        

    ##
    # @brief Add a transition function to sefl
    #
    # @param description The description of the transition function
    # @param state The state to transition to if function is satisfied
    # @param function The function to add
    def add_transition_function(self, description, function, state = None):
        self.transition_functions[description] = {"state" : state, "function" : function}

    ##
    # @brief Get description
    def get_desc(self):
        return self.state_desc

    ##
    # @brief Get the label of this state
    def get_label(self):
        return self.state_label

    ##
    # @brief Poll all transition functions and see if any have been satisfied, if not, return None
    #
    # @return The state to transition to
    def poll(self):
        triggered_transition_funcs = []
        move_on = False
        for i in self.transition_functions:
            ret = self.transition_functions[i]["function"]()
            if ret:
                triggered_transition_funcs.append(self.transition_functions[i]["state"])
                move_on = True
        if move_on:
            if len(triggered_transition_funcs) > 1:
                self.multiple_states_error(triggered_transition_funcs)
            return triggered_transition_funcs[0]
        else:
            return None

    ##
    # @brief An error raised if multiple transition functions have been activated
    #
    # @param states The states that are possible to go to
    def multiple_states_error(self, states):
        print("WARNING Invalid Deterministic Automota, multiple transition functions are active, picking the first active")
        print("The following states are validated: ")
        for i in states:
            print(i.get_label() + " : " + i.get_desc())

    def invalid_transition_function(self, desc):
        print("Invalid transition function with description %s" % (desc))

""" Example:
class Mining(State):
    ....

mining_state = Mining()

class Driving(State):
    def __init__(self, state_label: int, state_desc: str):
        super().__init__(state_label, state_desc)
        self.add_transition_function("sigma1", self.transition_function_one)

    def transition_function_one(self):
        return self.alphabet["driving"] == True

example_miner = SomeOtherState(2, "Example")

example_driver = Driving(1, "Driving robot")
example_driver.associate_transition_function("sigma1", example_driver) # when sigma1 transition is true, we move to example_driver

"""

class StateMachine:
    ##
    # @brief Initialize a State Machine
    #
    # @param starting_state The starting state of the state machine
    # @param alphabet_dict The dictionary definition of the alphabet
    # @param state_description The description of the state machine
    def __init__(self, starting_state: State, alphabet_dict: dict, state_description = "A State Machine"):
        self.staring_state = starting_state
        self.current_state = starting_state
        self.current_state.associate_alphabet(self.Alphabet)

        self.alphabet = Alphabet(alphabet_dict)
        self.state_description = state_description
        self.states = {}

    ##
    # @brief Initialize with a predefined list of states (i.e. you don't have to call add_state over and over)
    #
    # @param starting_state The starting state of the state machine
    # @param alphabet_dict The dictionary defined alphabet that we use for the state machine
    # @param states An array of State objects
    # @param state_description The description of the state machine
    def __init__(self, starting_state: State, alphabet_dict: dict, states, state_description = "A State Machine"):
        # Itterate through states and associate the alphabet with uurs
        self.states = {}
        self.alphabet = Alphabet(alphabet_dict)
        self.state_description = state_description

        for i in states:
            self.add_state(i)
        if starting_state.get_label() not in self.states:
            self.invalid_state(starting_state)
        else:
            self.current_state = starting_state
            self.staring_state = starting_state

    ##
    # @brief Adds a state to self
    #
    # @param state The state to add
    def add_state(self, state: State):
        state.associate_alphabet(self.alphabet)
        self.states[state.get_label()] = state


    ##
    # @brief An invalid state error most likely triggered if a state is not contained in the DSA
    #
    # @param state The invalid state
    def invalid_state(self, state):
        print("Invalid state %s" % (state.get_desc()))

    ##
    # @brief Gets the current state that we're on
    #
    # @return Our current state
    def get_state(self):
        return self.current_state
    
    ##
    # @brief Polls on the current state and transitions if needed
    def transition(self):
        next_state = self.current_state.poll()
        if next_state != None:
            self.current_state = next_state


""" Example:
states = 
[ Drive_State(0, "Driving"), Mine_State(1, "Mining"), Dead_State(2, "Dead")]
states[0].associate_transition_function("lambda1", states[1])

states[0].associate_transition_function("kill", states[2])
states[1].associate_transition_function("kill", states[2])

KermitStateMachine = StateMachine(states[0], kermit_alphabet, states, "A simple example statemachine")


while(1):
    KermitStateMachine.transition()
"""
