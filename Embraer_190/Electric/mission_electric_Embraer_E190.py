# mission_Embraer_E190_constThr.py
#
# Created:  Aug 2014, SUAVE Team
# Modified: Jun 2016, T. MacDonald 
#           Mar 2020, M. Clarke


""" setup file for a mission with a E190
"""


# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
import matplotlib.pyplot as plt  
import numpy as np  
import pylab as plt 

from SUAVE.Core import Units
from SUAVE.Plots.Mission_Plots import *  
from SUAVE.Core import Data, Container
from electric_Embraer_190 import vehicle_setup, configs_setup
from SUAVE.Methods.Performance  import payload_range
from SUAVE.Input_Output.Results import  print_parasite_drag,  \
     print_compress_drag, \
     print_engine_data,   \
     print_mission_breakdown

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():

    # define the problem
    configs, analyses = full_setup()

    configs.finalize()
    analyses.finalize()    

    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()      

    # mission analysis
    mission = analyses.missions
    results = mission.evaluate()

    # plt the old results
    plot_mission(results)

    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses, vehicle)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = mission

    return configs, analyses


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in list(configs.items()):
        analysis = base_analysis(config)
        analyses[tag] = analysis

    # adjust analyses for configs

    # takeoff_analysis
    analyses.takeoff.aerodynamics.drag_coefficient_increment = 0.1000

    # landing analysis
    aerodynamics = analyses.landing.aerodynamics
    # do something here eventually

    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.number_spanwise_vortices   = 4
    aerodynamics.settings.number_chordwise_vortices  = 2       
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()
    stability.geometry = vehicle
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Energy Analysis
    energy  = SUAVE.Analyses.Energy.Energy()
    energy.network=vehicle.propulsors
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    # done!
    return analyses    


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def mission_setup(analyses, vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'embraer_e190ar test mission'

    # atmospheric model
    atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    planet = SUAVE.Attributes.Planets.Earth()

    #airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments    

    # base segment
    base_segment = Segments.Segment()
    ones_row     = base_segment.state.ones_row
    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.state.numerics.number_control_points        = 4
    base_segment.process.iterate.unknowns.network            = vehicle.propulsors.battery_propeller.unpack_unknowns
    base_segment.process.iterate.residuals.network           = vehicle.propulsors.battery_propeller.residuals
    base_segment.state.unknowns.propeller_power_coefficient  = 0.005 * ones_row(1) 
    base_segment.state.unknowns.battery_voltage_under_load   = vehicle.propulsors.battery_propeller.battery.max_voltage * ones_row(1)  
    base_segment.state.residuals.network                     = 0. * ones_row(2)        


    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Throttle
    # ------------------------------------------------------------------

    # segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    # segment.tag = "climb_250kcas"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.takeoff )

    # # define segment attributes
    # segment.atmosphere     = atmosphere
    # segment.planet         = planet

    # segment.battery_energy = vehicle.propulsors.battery_propeller.battery.max_energy * 0.9

    # segment.altitude_start = 0.0   * Units.ft
    # segment.altitude_end   = 10000 * Units.ft
    # segment.air_speed      = 250.0 * Units.knots
    # segment.throttle       = 1.0        

    # # add to misison
    # mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Throttle
    # ------------------------------------------------------------------

    # segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    # segment.tag = "climb_280kcas"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.cruise )

    # # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    # segment.altitude_end = 32000. * Units.ft
    # segment.air_speed    = 350.0  * Units.knots
    # segment.throttle     = 1.0

    # # dummy for post process script
    # segment.climb_rate   = 0.1
    
    # ones_row = segment.state.ones_row
    # segment.state.unknowns.body_angle = ones_row(1) * 2. * Units.deg     

    # # add to mission
    # mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Climb Segment: Constant Speed, Constant Climb Rate
    # ------------------------------------------------------------------

    # segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    # segment.tag = "climb_final"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.cruise )

    # # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    # segment.altitude_end = 35000. * Units.ft
    # segment.air_speed    = 390.0  * Units.knots
    # segment.throttle     = 1.0

    # ones_row = segment.state.ones_row
    # segment.state.unknowns.body_angle = ones_row(1) * 2. * Units.deg   
    
    # # add to mission
    # mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Cruise Segment: constant speed, constant altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.atmosphere = atmosphere
    segment.planet     = planet

    segment.battery_energy = vehicle.propulsors.battery_propeller.battery.max_energy * 0.9

    segment.altitude = 9000. * Units.ft
    segment.air_speed  = 170. * Units.knots
    segment.distance   = 200. * Units.nmi
    segment.state.unknowns.body_angle = ones_row(1) * 2. * Units.deg 
    
    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Descent Segment: consant speed, constant segment rate
    # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    # segment.tag = "descent_m0_77"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.cruise )

    # # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    # segment.altitude_end = 30500  * Units.ft
    # segment.air_speed    = 440.0 * Units.knots
    # segment.descent_rate = 2600. * Units['ft/min']

    # # add to mission
    # mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment: consant speed, constant segment rate
    # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    # segment.tag = "descent_290kcas"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.cruise )

    # # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    # segment.altitude_end = 12000 * Units.ft
    # segment.air_speed    = 365.0 * Units.knots
    # segment.descent_rate = 2000. * Units['ft/min']

    # # append to mission
    # mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Descent Segment: consant speed, constant segment rate
    # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    # segment.tag = "descent_250kcas"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.cruise )

    # # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    # segment.altitude_end = 0.0   * Units.ft
    # segment.air_speed    = 250.0 * Units.knots
    # segment.descent_rate = 1500. * Units['ft/min']

    # # append to mission
    # mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Mission definition complete
    # ------------------------------------------------------------------

    return mission
 
# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):

    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style) 

    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results, line_style) 

    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results, line_style)

    # Plot Static Stability Coefficients 
    plot_stability_coefficients(results, line_style)  

    # Drag Components
    plot_drag_components(results, line_style)

    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results, line_style)

    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results, line_style)   

    # Plot Aircraft Electronics
    plot_electronic_conditions(results, line_style)
    
    # Plot Propeller Conditions 
    plot_propeller_conditions(results, line_style) 
    
    # Plot Electric Motor and Propeller Efficiencies 
    plot_eMotor_Prop_efficiencies(results, line_style)
    
    # Plot propeller Disc and Power Loading
    plot_disc_power_loading(results, line_style) 

    return

if __name__ == '__main__':
    main()
    plt.show()