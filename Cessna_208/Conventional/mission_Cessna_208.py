# mission_Caravan.py
# 
# Created:    Ago 2018, M. Gallani
# Modified:   Jun 2021, V. Tanigawa


#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import SUAVE
from SUAVE.Analyses.Mission.Segments.Conditions.State import State
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
    mission  = mission_setup(configs_analyses, vehicle)
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
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    weights.settings.empty_weight_increment = 0.
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0128
    aerodynamics.settings.oswald_efficiency_factor   = 0.7860  ## Oswald for the case considering thrust effect on fuselage drag
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
    config.wings['main_wing'].control_surfaces.flap.deflection = 15. * Units.deg
    # A max lift coefficient factor of 1 is the default, but it is highlighted here as an option
    config.max_lift_coefficient_factor    = 1.

    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.max_lift_coefficient_factor    = 1.

    configs.append(config)    

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.max_lift_coefficient_factor    = 1. 

    configs.append(config)

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'
    
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
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
def mission_setup(analyses,vehicle):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # Airport
    # The airport parameters are used in calculating field length and noise. They are not
    # directly used in mission performance estimation
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # Unpack Segments Module
    Segments = SUAVE.Analyses.Mission.Segments

    # Base Segment 
    base_segment = Segments.Segment()
    ones_row = base_segment.state.ones_row
    base_segment.process.iterate.unknowns.network = vehicle.propulsors.internal_combustion.unpack_unknowns
    base_segment.process.iterate.residuals.network = vehicle.propulsors.internal_combustion.residuals
    base_segment.state.unknowns.pitch_command = ones_row(1) * 0. * Units.deg  
    base_segment.state.residuals.net = 0. * ones_row(1)
    base_segment.state.numerics.number_control_points = 4
    base_segment.state.unknowns.throttle                 = 0.1   *  ones_row(1)
    base_segment.state.unknowns.rpm                      = 1900. *  ones_row(1) 
    base_segment.state.residuals.network                 = 0.    * ones_row(1)


    # # ------------------------------------------------------------------
    # #   First Climb Segment: Constant Speed, Constant Rate 
    # # ------------------------------------------------------------------

    # # A constant speed, constant rate climb segment is used first. This means that the aircraft
    # # will maintain a constant airspeed and constant climb rate until it hits the end altitude.
    # # For this type of segment, the throttle is allowed to vary as needed to match required
    # # performance.
    # segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    # # It is important that all segment tags must be unique for proper evaluation. At the moment 
    # # this is not automatically enforced. 
    # segment.tag = "climb_1"

    # # The analysis settings for mission segment are chosen here. These analyses include information
    # # on the vehicle configuration.
    # segment.analyses.extend( analyses.takeoff )

    # segment.altitude_start = 0.0   * Units.ft
    # segment.altitude_end   = 3000.0   * Units.ft
    # segment.air_speed      = 125.0 * Units['kts']
    # segment.climb_rate     = 800   * Units.ft / Units.min

    # # Add to misison
    # mission.append_segment(segment)


    # # ------------------------------------------------------------------
    # #   Second Climb Segment: Constant Speed, Constant Rate 
    # # ------------------------------------------------------------------

    # segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment) 
    # segment.tag = "climb_2"

    # segment.analyses.extend( analyses.takeoff )

    # # A starting altitude is no longer needed as it will automatically carry over from the
    # # previous segment. However, it could be specified if desired. This would potentially cause
    # # a jump in altitude but would otherwise not cause any problems.
    # segment.altitude_end   = 6000.0   * Units.ft
    # segment.air_speed      = 150.0 * Units['kts']
    # segment.climb_rate     = 800   * Units.ft / Units.min

    # # Add to misison
    # mission.append_segment(segment)


    # # ------------------------------------------------------------------
    # #   Third Climb Segment: Constant Speed, Constant Rate 
    # # ------------------------------------------------------------------

    # segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    # segment.tag = "climb_3"

    # segment.analyses.extend( analyses.takeoff )

    # segment.altitude_end   = 9000.0   * Units.ft
    # segment.air_speed      = 160.0 * Units['kts']
    # segment.climb_rate     = 400   * Units.ft / Units.min

    # # Add to misison
    # mission.append_segment(segment)


    # # ------------------------------------------------------------------    
    # #  Cruise Segment: Constant Speed Constant Altitude
    # # ------------------------------------------------------------------    

    # segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    # segment.tag = "cruise"

    # segment.analyses.extend( analyses.cruise )

    # segment.air_speed = 170 * Units['kts'] 
    # segment.distance  = 200 * Units.nmi
    
    # segment.state.numerics.number_control_points = 10

    # # add to mission
    # mission.append_segment(segment)


    # Calibration --------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )

    segment.altitude  = 20000 * Units.ft
    segment.air_speed = 163 * Units['kts'] 
    segment.distance  = 200 * Units.nmi
    
    segment.state.numerics.number_control_points = 10

    # add to mission
    mission.append_segment(segment)


    # # ------------------------------------------------------------------
    # #   First Descent Segment: Constant Speed, Constant Rate
    # # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    # segment.tag = "descent_1"

    # segment.analyses.extend( analyses.cruise )

    # segment.altitude_end = 7200.0   * Units.ft
    # segment.air_speed    = 160.0 * Units['kts']
    # segment.descent_rate   = 450.   * Units.ft / Units.min

    # # Add to mission
    # mission.append_segment(segment)

    # # ------------------------------------------------------------------
    # #   Second Descent Segment: Constant Speed, Constant Rate
    # # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    # segment.tag = "descent_2"

    # segment.analyses.extend( analyses.landing )

    # segment.altitude_end = 5400.0   * Units.ft
    # segment.air_speed    = 155.0 * Units['kts']
    # segment.descent_rate   = 500.   * Units.ft / Units.min

    # # Add to mission
    # mission.append_segment(segment)

    # # ------------------------------------------------------------------
    # #   Third Descent Segment: Constant Speed, Constant Rate
    # # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    # segment.tag = "descent_3"

    # segment.analyses.extend( analyses.landing )
    # # While it is set to zero here and therefore unchanged, a drag increment can be used if
    # # desired. This can avoid negative throttle values if drag generated by the base airframe
    # # is insufficient for the desired descent speed and rate.
    # analyses.landing.aerodynamics.settings.spoiler_drag_increment = 0.00

    # segment.altitude_end = 3600.0   * Units.ft
    # segment.air_speed    = 150.0 * Units['kts']
    # segment.descent_rate   = 500.   * Units.ft / Units.min

    # # Add to mission
    # mission.append_segment(segment)

    # # ------------------------------------------------------------------
    # #   Fourth Descent Segment: Constant Speed, Constant Rate
    # # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    # segment.tag = "descent_4"

    # segment.analyses.extend( analyses.landing )
    # analyses.landing.aerodynamics.settings.spoiler_drag_increment = 0.00

    # segment.altitude_end = 1800.0   * Units.ft
    # segment.air_speed    = 145.0 * Units['kts']
    # segment.descent_rate   = 500.   * Units.ft / Units.min

    # # Add to mission
    # mission.append_segment(segment)

    # # ------------------------------------------------------------------
    # #   Fifth Descent Segment: Constant Speed, Constant Rate
    # # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    # segment.tag = "descent_5"

    # segment.analyses.extend( analyses.landing )
    # analyses.landing.aerodynamics.settings.spoiler_drag_increment = 0.00

    # segment.altitude_end = 0.0   * Units.ft
    # segment.air_speed    = 140.0 * Units['kts']
    # segment.descent_rate   = 300.   * Units.ft / Units.min

    # # Append to mission
    # mission.append_segment(segment)

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
def plot_mission(results, line_style = 'bo-'):

    # # Plot Flight Conditions 
    # plot_flight_conditions(results, line_style) 

    # # Plot Aerodynamic Coefficients
    # plot_aerodynamic_coefficients(results, line_style) 

    # # Plot Aerodynamic Forces 
    # plot_aerodynamic_forces(results, line_style)

    # # Plot Static Stability Coefficients 
    # plot_stability_coefficients(results, line_style)  

    # # Drag Components
    # plot_drag_components(results, line_style)

    # # Plot Aircraft Flight Speed
    # plot_aircraft_velocities(results, line_style)

    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results, line_style)  

    # plot_disc_power_loading(results, line_style) 

    # plot_propeller_conditions(results, line_style)

    # plot_surface_pressure_contours(results, line_style)

    # plot_lift_distribution(results, line_style)

    # create_video_frames(results, line_style)

    # plot_noise_level(results, line_style)

    # plot_flight_profile_noise_contour(results, line_style)
    
    return 

if __name__ == '__main__':
    main()
    
    plt.show()