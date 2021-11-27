## @ingroup Components-Energy-Networks
# Simple_Propulsor.py
# 
# Created:  Ago 2018, M. Gallani


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE

# package imports
import numpy as np
from SUAVE.Components.Propulsors.Propulsor import Propulsor

from SUAVE.Core import Data, Units

# ----------------------------------------------------------------------
#  Network
# ----------------------------------------------------------------------

## @ingroup Components-Energy-Networks
class PT6_Turboprop(Propulsor):
    """ A simple prpulsor network that just outputs thrust as a function of a Max Thrust and throttle settings.
        
        Unknowns:
            Throttle
    
        Assumptions:
        
        Source:
        None
    """      
    def __defaults__(self):
        """ This sets the default values for the network to function.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            None
    
            Outputs:
            None
    
            Properties Used:
            N/A
        """            

        self.nacelle_diameter  = None
        self.engine_length     = None
        self.number_of_engines = None
        
        self.propeller_speed    = 0.
        
        self.engine            = None
        self.propeller         = None
        self.tag               = 'network'

    
    # manage process with a driver function
    def evaluate_thrust(self,state):
        """ Calculate thrust given the current state of the vehicle
    
            Assumptions:
                
            Source:
            N/A
    
            Inputs:
            state [state()]
    
            Outputs:

    
            Properties Used:
            Defaulted values
        """          
    
        # unpack
        conditions  = state.conditions
        eta         = conditions.propulsion.throttle[:,0,None]
        omega       = self.propeller_speed*np.ones_like(eta)
        pitch       = conditions.propulsion.pitch_command
        
        eng     = self.engine
        prop    = self.propeller
        
        ## Calculate Engine Performance
        eng.inputs.omega    = omega
        eng.inputs.throttle = eta
        eng.evaluate(conditions)
        mdot        = eng.outputs.fuel_flow
        eng_power   = eng.outputs.brake_horsepower
        
        ## Calculate Propeller Performance
        prop.inputs.omega           = omega
        prop.inputs.pitch_command   = pitch
        T, Q, prop_power, Cp = prop.spin(conditions)
        
       
        #Create the outputs
#        F    = self.number_of_engines * T * [np.cos(self.thrust_angle),0,-np.sin(self.thrust_angle)] 
        F    =  T * [np.cos(self.thrust_angle),0,-np.sin(self.thrust_angle)] 
        
        conditions.propulsion.thrust    = F
        conditions.propulsion.fuel_flow = mdot
        conditions.propulsion.sfc       = eng.outputs.sfc
        
        conditions.propulsion.engine_power = eng_power
        conditions.propulsion.propeller_power = prop_power
        conditions.propulsion.propeller_omega = omega
        
        results = Data()
        results.thrust_force_vector = F
#        results.vehicle_mass_rate   = mdot
        results.vehicle_mass_rate = 0.

        return results
    
    def weight(self):
        """ Calculates the weight of the propulsion system
            Does not include battery weight, payload nor avionics.
            Battery weight should be defined and calculated individually
            Avionics and payload weight should be included on the empty weight script 
        """
        
        # List of which components' weights shall be calculated and included on the propulsive system weight
        items_list = ['engine','propeller']
        
        # Initialize weight at zero
        weight = 0
        
        # Run the list of items and calculate the weight for each one
        # Add everything to come up with the total system weight
        for i in items_list:
            W = self[i].weight()
            weight += W
        
        # Output the weight
        self.mass_properties.mass = weight
        
        return weight
    
    
    def unpack_unknowns(self,segment,state):
        """ This is an extra set of unknowns which are unpacked from the mission solver and send to the network.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            state.unknowns.propeller_power_coefficient [None]
    
            Outputs:
            state.conditions.propulsion.propeller_power_coefficient [None]
    
            Properties Used:
            N/A
        """    
        
        state.conditions.propulsion.pitch_command  = state.unknowns.pitch_command
        
        return
    
    def residuals(self,segment,state):
        """ This packs the residuals to be send to the mission solver.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            state.conditions.propulsion:
                motor_torque                          [N-m]
                propeller_torque                      [N-m]
            
            Outputs:
            None
    
            Properties Used:
            None
        """  
        prop_power  = state.conditions.propulsion.propeller_power
        eng_power   = state.conditions.propulsion.engine_power
        
        # Here we are going to pack the residuals from the network
        # Equation 1: Power balance between motor and propeller
        state.residuals.network[:,0] = prop_power[:,0] - eng_power[:,0]
        state.residuals.network[:,1] = state.unknowns.dep_throttle[:,0] - 0.
        
        return        
            
    __call__ = evaluate_thrust