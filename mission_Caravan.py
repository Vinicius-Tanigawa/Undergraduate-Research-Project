# mission_Caravan.py
# 
# Created:    Ago 2018, M. Gallani
# Modified:   Jun 2021, V. Tanigawa


#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import SUAVE
import numpy as np
import pylab as plt
import time

from SUAVE.Core import Units, Data
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Plots.Mission_Plots import *
from SUAVE.Plots.Geometry_Plots import * 

from Cessna_208 import vehicle_setup, configs_setup

from SUAVE.Input_Output.Results import  print_parasite_drag,  \
     print_compress_drag, \
     print_engine_data,   \
     print_mission_breakdown, \
     print_weight_breakdown


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    """This function gets the vehicle configuration, analysis settings, and then runs the mission.
    Once the mission is complete, the results are plotted."""
    
    # Extract vehicle configurations and the analysis settings that go with them
    configs, analyses = full_setup()

    # Size each of the configurations according to a given set of geometry relations
    simple_sizing(configs)

    # Perform operations needed to make the configurations and analyses usable in the mission
    configs.finalize()
    analyses.finalize()

    # Determine the vehicle weight breakdown (independent of mission fuel usage)
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()      

    # Perform a mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # Plot all mission results, including items such as altitude profile and L/D
    plot_mission(results)

    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():
    """This function gets the baseline vehicle and creates modifications for different 
    configurations, as well as the mission and analyses to go with those configurations."""

    # Collect baseline vehicle data and changes when using different configuration settings
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)

    # Get the analyses to be used when different configurations are evaluated
    configs_analyses = analyses_setup(configs)

    # Create the mission that will be flown
    mission  = mission_setup(configs_analyses)
    missions_analyses = missions_setup(mission)

    # Add the analyses to the proper containers
    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = SUAVE.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):
    """This is the baseline set of analyses to be used with this vehicle. Of these, the most
    commonly changed are the weights and aerodynamics methods."""

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_UAV()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()
    stability.geometry = vehicle
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors 
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

    return analyses    


# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    """This function sets up vehicle configurations for use in different parts of the mission.
    Here, this is mostly in terms of high lift settings."""
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------
    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    configs.append(config)

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
#    config.wings['main_wing'].control_surfaces.slat.deflection = 25. * Units.deg
    # A max lift coefficient factor of 1 is the default, but it is highlighted here as an option
    config.max_lift_coefficient_factor    = 1.

    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
#    config.wings['main_wing'].control_surfaces.slat.deflection = 20. * Units.deg
    config.max_lift_coefficient_factor    = 1.

    configs.append(config)    

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
#    config.wings['main_wing'].control_surfaces.slat.deflection = 25. * Units.deg  
    config.max_lift_coefficient_factor    = 1. 

    configs.append(config)

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'
    
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
#    config.wings['main_wing'].control_surfaces.slat.deflection = 20. * Units.deg
    config.max_lift_coefficient_factor    = 1. 
  
    configs.append(config)

    return configs

def simple_sizing(configs):
    """This function applies a few basic geometric sizing relations and modifies the landing
    configuration."""

    base = configs.base
    # Update the baseline data structure to prepare for changes
    base.pull_base()

    # Revise the zero fuel weight. This will only affect the base configuration. To do all
    # configurations, this should be specified in the top level vehicle definition.
    base.mass_properties.max_zero_fuel = 0.9 * base.mass_properties.max_takeoff 

    # Estimate wing areas
    for wing in base.wings:
        wing.areas.wetted   = 2.0 * wing.areas.reference
        wing.areas.exposed  = 0.8 * wing.areas.wetted
        wing.areas.affected = 0.6 * wing.areas.wetted

    # Store how the changes compare to the baseline configuration
    base.store_diff()

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    landing = configs.landing

    # Make sure base data is current
    landing.pull_base()

    # Add a landing weight parameter. This is used in field length estimation and in
    # initially the landing mission segment type.
    landing.mass_properties.landing = 0.85 * base.mass_properties.takeoff

    # Store how the changes compare to the baseline configuration
    landing.store_diff()

    return


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

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


    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

#    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
#    segment.tag = "climb_1"
#
#    segment.analyses.extend( analyses.takeoff )
#
#    segment.altitude_start = 0.0   * Units.km # modified
#    segment.altitude_end   = 1.0   * Units.km # modified
#    segment.air_speed      = 36.011 * Units['m/s'] # modified
#    segment.climb_rate     = 3.912   * Units['m/s'] # modified
# 
#    # add to misison
#    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

#    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
#    segment.tag = "climb_2"
#
#    segment.analyses.extend( analyses.cruise )
#
#    segment.altitude_end   = 2.3   * Units.km # modified
#    segment.air_speed      = 41.156 * Units['m/s'] # modified
#    segment.climb_rate     = 3.912   * Units['m/s'] # modified
#
#    # add to mission
#    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

#    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
#    segment.tag = "climb_3"
#
#    segment.analyses.extend( analyses.cruise )
#
#    segment.altitude_end = 3.652   * Units.km # modified
#    segment.air_speed    = 51.444  * Units['m/s'] # modified
#    segment.climb_rate   = 3.912    * Units['m/s'] # modified
#
#    # add to mission
#    mission.append_segment(segment)


    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )

    segment.altitude  = 10000. * Units.ft # modified
    segment.air_speed = 184 * Units['kts'] # modified
    segment.distance  = (1200) * Units.km # modified
    
    segment.state.numerics.number_control_points = 10

    # add to mission
    mission.append_segment(segment)


# ------------------------------------------------------------------
#   First Descent Segment: Constant Speed Constant Rate  
# ------------------------------------------------------------------

#    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
#    segment.tag = "descent_1"
#
#    segment.analyses.extend( analyses.cruise )
#
#    segment.altitude_start = 3.652 * Units.km # modified
#    segment.altitude_end   = 2.922   * Units.km # modified
#    segment.air_speed      = 51.444 * Units['m/s'] # modified
#    segment.descent_rate   = 2.934   * Units['m/s'] # modified
#
#    # add to mission
#    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

#    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
#    segment.tag = "descent_2"
#
#    segment.analyses.extend( analyses.landing )
#
#    analyses.landing.aerodynamics.settings.spoiler_drag_increment = 0.00
#
#    segment.altitude_end = 2.191   * Units.km # modified
#    segment.air_speed    = 41.156 * Units['m/s'] # modified
#    segment.descent_rate = 3.26   * Units['m/s'] # modified
#
#    # add to mission
#    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

#    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
#    segment.tag = "descent_3"
#
#    segment.analyses.extend( analyses.landing )
#
#    analyses.landing.aerodynamics.settings.spoiler_drag_increment = 0.00
#
#    segment.altitude_end = 1.461   * Units.km # modified
#    segment.air_speed    = 38.583 * Units['m/s'] # modified
#    segment.descent_rate = 3.26   * Units['m/s'] # modified
#
#    # add to mission
#    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Fourth Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

#    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
#    segment.tag = "descent_4"
#
#    segment.analyses.extend( analyses.landing )
#
#    analyses.landing.aerodynamics.settings.spoiler_drag_increment = 0.00
#
#    segment.altitude_end = 0.730   * Units.km # modified
#    segment.air_speed    = 36.011 * Units['m/s'] # modified
#    segment.descent_rate = 3.26   * Units['m/s'] # modified
#
#
#    # add to mission
#    mission.append_segment(segment)



    # ------------------------------------------------------------------
    #   Fifth Descent Segment:Constant Speed Constant Rate  
    # ------------------------------------------------------------------

#    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
#    segment.tag = "descent_5"
#
#    segment.analyses.extend( analyses.landing )
#    analyses.landing.aerodynamics.settings.spoiler_drag_increment = 0.00
#
#
#    segment.altitude_end = 0.0   * Units.km # modified
#    segment.air_speed    = 33.439 * Units['m/s'] # modified
#    segment.descent_rate = 1.956   * Units['m/s'] # modified
#
#
#    # append to mission
#    mission.append_segment(segment)

    return mission

def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()
    
    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------
    
    missions.base = base_mission
    
    # done!
    return missions  

# ----------------------------------------------------------------------
#   Plot Results
# ----------------------------------------------------------------------
def plot_mission(results):

    # Plot Flight Conditions 
    plot_flight_conditions(results) 

    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results) 

    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results)

    # Plot Static Stability Coefficients 
    plot_stability_coefficients(results)  

    # Drag Components
    plot_drag_components(results)

    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results)

    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results)  

    # Plot Propeller Conditions 
#    plot_propeller_conditions(results) 

    
    return 

if __name__ == '__main__':
    main()
    
    plt.show()