# Ipanema_single_analysis.py
# 
# Created:  Ago 2018, M. Gallani


#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data

import numpy as np
import pylab as plt
import time
from scipy.optimize import curve_fit

from SUAVE.Methods.Utilities import plot_format
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Utilities.wing_sizing import chord_calc, Saff_calc

from Missions import mission_setup, mission_setup2, mission_setup3, mission_setup4, mission_setup5, mission_setup6

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    
    # build the vehicle, configs, and analyses
    configs, analyses = full_setup()
    
    configs.finalize()
    analyses.finalize()    
    
    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()          
    
    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()
    
    # plot results    
    plot_mission(results)
    
    return breakdown, results, configs.base

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():
    
    # vehicle data
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)
    
    # vehicle analyses
    configs_analyses = analyses_setup(configs)
    
#     mission analyses
    mission  = mission_setup4(configs_analyses,vehicle)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses
    
    return configs, analyses

# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Caravan 208'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------


    vehicle.mass_properties.max_takeoff               = 8750 * Units['lb'] 
#    vehicle.mass_properties.ramp                      = 8750 * Units['lb']- 368.8
    vehicle.mass_properties.ramp                      = 8750 * Units['lb']
    vehicle.mass_properties.takeoff                   = vehicle.mass_properties.ramp
    vehicle.mass_properties.cargo                     = 0  * Units.kg
    vehicle.mass_properties.max_landing               = 8750 * Units['lb'] 
    

    # basic parameters
    vehicle.reference_area         = 25.96
    vehicle.passengers             = 10.
    
    vehicle.envelope.ultimate_load  = 3.8
    
    vehicle.max_lift_coefficient_factor = 0.87889
    

    #------------------------------------------------------------------
    # Propulsor
    #------------------------------------------------------------------
    
    # build network
    net = SUAVE.Components.Energy.Networks.PT6_Turboprop()
    net.number_of_engines = 1.
    net.nacelle_diameter  = 0.2 * Units.meters
    net.engine_length     = 0.01 * Units.meters
    net.areas             = Data()
    net.areas.wetted      = 0
    
    net.propeller_speed = 1900 * Units['rpm']
    net.thrust_angle    = 0.
    
    
    # Create Engine
    eng = SUAVE.Components.Energy.Converters.PT6()
    eng.rated_power     = 675*Units['hp']
    eng.rated_speed     = 1900 * Units['rpm']
    eng.torque_limit    = 1970 * Units['lbf*ft']
    eng.rated_sfc       = 0.64 * Units['lb/hp/h'] * 1.02
    net.engine = eng
    
    
    # Create Propeller
    prop_attributes = Data()
    prop_attributes.number_blades       = 3.0
#    prop_attributes.freestream_velocity = 136 * Units['kts']# freestream
    prop_attributes.freestream_velocity = 90 * Units['kts']# freestream
    prop_attributes.angular_velocity    = 1900. * Units['rpm']
    prop_attributes.tip_radius          = 53  * Units['in']
    prop_attributes.hub_radius          = 0.15 * Units.meters
#    prop_attributes.design_Cl           = 0.7
    prop_attributes.design_Cl           = 0.4
    prop_attributes.design_altitude     = 5000 * Units.ft
    prop_attributes.design_thrust       = 0.0 
    prop_attributes.design_power        = 600.0 * Units['hp']
    prop_attributes.activity_factor     = 90
    prop_attributes.weight_class        = 0
    prop_attributes                     = propeller_design(prop_attributes)
    
    prop = SUAVE.Components.Energy.Converters.Propeller_General()
    prop.prop_attributes = prop_attributes
    net.propeller        = prop
        
    vehicle.append_component(net)
    
    
    
    
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'
    fuselage.areas.wetted           = 75.58
    fuselage.lengths.total          = 16.67
    fuselage.width                  = 1.71
    fuselage.heights.maximum        = 2.10 + 0.68
    fuselage.areas.front_projected  = fuselage.width*fuselage.heights.maximum
    fuselage.effective_diameter     = np.sqrt(4*fuselage.areas.front_projected/np.pi)
    fuselage.fineness_ratio         = fuselage.lengths.total/fuselage.effective_diameter
    

    fuselage.strut = Data()
    fuselage.strut.thickness_to_chord = 0.12
    fuselage.strut.chord    = 0.3
    fuselage.strut.length   = 2.9 
    
    fuselage.propeller_radius = prop.prop_attributes.tip_radius

    # add to vehicle
    vehicle.append_component(fuselage)

    # ------------------------------------------------------------------
    #  Main Wing
    # ------------------------------------------------------------------
 
    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'


    wing.areas.reference         = vehicle.reference_area
    wing.spans.projected         = 15.875
     
    wing.aspect_ratio            = wing.spans.projected**2/wing.areas.reference 
    wing.thickness_to_chord      = 0.15 # average
    
    wing.sweeps.quarter_chord    = 2.93 * Units.deg
    wing.taper                   = 0.586
#    wing.sweeps.quarter_chord    = 0 * Units.deg
#    wing.taper                   = 1.0

    wing.number_of_motors        = 0.

#    wing.chords.root             = 2.22
#    wing.chords.tip              = 1.41
#    wing.chords.mean_aerodynamic = (wing.chords.root + wing.chords.tip)/2
    
    wing.chords.root             = 2*wing.areas.reference/(wing.spans.projected*(1+wing.taper))
    wing.chords.tip              = wing.chords.root*wing.taper
    wing.chords.mean_aerodynamic = (wing.chords.root + wing.chords.tip)/2
    
    wing.areas.wetted            = wing.wetted_area(fuselage)

    wing.twists.root             =  0.0 * Units.degrees
    wing.twists.tip              =  0.0 * Units.degrees

    wing.vertical                = False
    wing.symmetric               = True

    wing.high_lift               = True
    wing.flaps.type              = "single_slotted"
    wing.flaps.angle             = 0.0 * Units['degree']
    wing.flaps.chord             = 0.41
    wing.flaps.span              = 4.84
    wing.flaps.span_start        = fuselage.width/2
    wing.flaps.span_end          = wing.flaps.span_start + wing.flaps.span
    wing.flaps.chord_start       = chord_calc(fuselage.width/2, wing.chords.root, wing.chords.tip, wing.spans.projected/2.)
#    wing.flaps.chord_end         = 1.76
    wing.flaps.chord_end         = chord_calc(wing.flaps.span_end, wing.chords.root, wing.chords.tip, wing.spans.projected/2.)
#    Saffected = (wing.flaps.chord_start + wing.flaps.chord_end) * (wing.flaps.span)
    Saffected = Saff_calc(wing.flaps.span_end, wing.chords.root, wing.chords.tip, wing.spans.projected/2, fuselage.width/2)
    wing.areas.affected          = Saffected #/ wing.areas.reference

    wing.unblown_maximum_lift_coefficient = 1.521365534   

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 2.95
    wing.sweeps.quarter_chord    = 2.05 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.62

    wing.spans.projected         = 6.25

    wing.chords.root             = 1.31
    wing.chords.tip              = 0.81
    wing.chords.mean_aerodynamic = (wing.chords.root + wing.chords.tip)/2

    wing.areas.reference         = 13.22
    wing.areas.wetted            = 2*(wing.areas.reference - 0.50*wing.chords.root)

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.vertical                = False
    wing.symmetric               = True


    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'vertical_stabilizer'

    wing.aspect_ratio            = 2.0      #
    wing.sweeps.quarter_chord    = 14.06 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.38

    wing.spans.projected         = 3.05

    wing.chords.root             = 2.21
    wing.chords.tip              = 0.84
    wing.chords.mean_aerodynamic = (wing.chords.root + wing.chords.tip)/2

    wing.areas.reference         = 4.21
    wing.areas.wetted            = 2.0 * wing.areas.reference

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.vertical                = True
    wing.symmetric               = False

    # add to vehicle
    vehicle.append_component(wing)


    
    #now other objects
    mlg=SUAVE.Components.Landing_Gear.Landing_Gear()
    mlg.tag = 'main_ldg'
    mlg.CD              = 0.615 # Gudmundsson J1
    mlg.tire.diameter   = 0.6
    mlg.tire.width      = 0.2
    vehicle.append_component(mlg)
    
    tlg=SUAVE.Components.Landing_Gear.Landing_Gear()
    tlg.tag = 'nose_ldg'
    tlg.CD              = 1.9 # Gudmundsson Q
    tlg.tire.diameter   = 0.5
    tlg.tire.width      = 0.18
    vehicle.append_component(tlg)
    


    return vehicle
    # -----------------------------------------------
    #   Vehicle Definition Complete
    # -----------------------------------------------

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)
    
    # ------------------------------------------------------------------
    #   Takeoff Configurations
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff1'
    config.wings.main_wing.flaps.angle = 20*Units['degree']
    config.propulsors.network.propeller_speed = 1900*Units['rpm']
    configs.append(config)
    
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff2'
    config.wings.main_wing.flaps.angle = 10*Units['degree']
    config.propulsors.network.propeller_speed = 1900*Units['rpm']
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Cruise Configurations
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    config.wings.main_wing.flaps.angle = 00*Units['degree']
    configs.append(config)


    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise2' # for calibration segments
    config.wings.main_wing.flaps.angle = 00*Units['degree']
    config.propulsors.network.propeller_speed = 1750*Units['rpm']
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Approach Configurations
    # ------------------------------------------------------------------
    
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'approach1'
    config.wings.main_wing.flaps.angle = 30*Units['degree']
    config.propulsors.network.propeller_speed = 1750*Units['rpm']
    configs.append(config)
    
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'approach2'
    config.wings.main_wing.flaps.angle = 30*Units['degree']
    config.propulsors.network.propeller_speed = 1750*Units['rpm']
    configs.append(config)
    
    
    # ------------------------------------------------------------------
    #   Stall Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'stall'
    config.wings.main_wing.flaps.angle = 30*Units['degree']
    config.propulsors.network.propeller_speed = 1900*Units['rpm']    
    configs.append(config)
    
    return configs
    


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):
    
    analyses = SUAVE.Analyses.Analysis.Container()
    
    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis
    
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
    weights = SUAVE.Analyses.Weights.Weights_Caravan()
    weights.vehicle = vehicle
    weights.settings.empty_weight_increment = 0.
    analyses.append(weights)
    
    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Caravan_Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0004
    aerodynamics.settings.oswald_efficiency_factor   = 0.7860  ## Oswald for the case considering thrust effect on fuselage drag
#    aerodynamics.settings.oswald_efficiency_factor   = None
    aerodynamics.settings.interference.wing          = 1.05 #1.15
    aerodynamics.settings.interference.strut         = 1.10 #1.1
    aerodynamics.settings.interference.landing_gear  = 1.0 #1.35
    
    aerodynamics.settings.miscellaneous.fuselage        = 1.0 #1.67
    aerodynamics.settings.miscellaneous.wing            = 1.0 #1.15
    aerodynamics.settings.miscellaneous.landing_gear    = 1.0 #1.35
    aerodynamics.settings.miscellaneous.excrescences    = 2.95 #1.0
    analyses.append(aerodynamics)
    
    # ------------------------------------------------------------------
    #  Energy
    energy = SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors #what is called throughout the mission (at every time step))
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
    plt.close('all')

        
#     ------------------------------------------------------------------    
#       Aerodynamics
#     ------------------------------------------------------------------
#    fig = plt.figure("Aerodynamic Forces")
#    for segment in results.segments.values():
#        
#        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
#        Lift   = -segment.conditions.frames.wind.lift_force_vector[:,2] / Units.N
#        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0]
#        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0]
#
#        axes = fig.add_subplot(3,1,1)
#        axes.plot( time , Lift , 'bo-' )
#        axes.set_xlabel('Time (min)')
#        axes.set_ylabel('Lift (N)')
#        axes.get_yaxis().get_major_formatter().set_scientific(False)
#        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
#        axes.grid(True)
#        
#        axes = fig.add_subplot(3,1,2)
#        axes.plot( time , Drag , 'bo-' )
#        axes.set_xlabel('Time (min)')
#        axes.set_ylabel('Drag (N)')
#        axes.get_yaxis().get_major_formatter().set_scientific(False)
#        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
#        axes.grid(True)
#        
#        axes = fig.add_subplot(3,1,3)
#        axes.plot( time , Thrust , 'bo-' )
#        axes.set_xlabel('Time (min)')
#        axes.set_ylabel('Thrust (N)')
#        axes.get_yaxis().get_major_formatter().set_scientific(False)
#        axes.get_yaxis().get_major_formatter().set_useOffset(False)  
##        plt.ylim((0,50))
#        axes.grid(True)
        
#     ------------------------------------------------------------------    
#       Aerodynamics 2
#     ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Coefficients")
    for segment in results.segments.values():
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        CLift  = segment.conditions.aerodynamics.lift_coefficient[:,0]
        CDrag  = segment.conditions.aerodynamics.drag_coefficient[:,0]
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0]
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0]

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , CLift , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('CL')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True, which = 'both')
        
        axes = fig.add_subplot(3,1,2)
        axes.plot( time , CDrag , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('CD')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True, which = 'both')
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , Drag   , 'bo-' )
        axes.plot( time , Thrust , 'ro-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Drag and Thrust (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)  
        axes.grid(True, which = 'both')
        
#     ------------------------------------------------------------------    
#       Aerodynamic Efficiency
#     ------------------------------------------------------------------
    fig = plt.figure("LD")
    axes = plt.gca()    
    for i, segment in enumerate(results.segments.values()):
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        CLift  = segment.conditions.aerodynamics.lift_coefficient[:,0]
        CDrag  = segment.conditions.aerodynamics.drag_coefficient[:,0]
        LD = CLift/CDrag
        axes.plot(time, LD, 'o-', color = colors[0], linewidth = thickness)
        plot_format(fig, axes, xlabel = 'Time [mins]',              xlabel_size = xlabel_size,\
                           ylabel = 'L/D [-]',             ylabel_size = ylabel_size,\
                           title  = 'Aerodynamic Efficiency',           title_size  = title_size,\
                           tick_size = tick_size,               tick_rotation = tick_rotation,\
                           grid = grid,                         minor_ticks = minor_ticks) 
    
#     ------------------------------------------------------------------    
#       Aerodynamics 2
#     ------------------------------------------------------------------
    fig = plt.figure("Drag Components")
    axes = plt.gca()    
    for i, segment in enumerate(results.segments.values()):
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        cdp = drag_breakdown.parasite.total[:,0]
        cdi = drag_breakdown.induced.total[:,0]
        cd  = drag_breakdown.total[:,0]
        
        axes.plot( time , cdp , 'ko-', label='CD_P' )
        axes.plot( time , cdi , 'bo-', label='CD_I' )
        axes.plot( time , cd  , 'ro-', label='CD'   )
        
        if i == 0:
            axes.legend(loc='upper center')
        
    axes.set_xlabel('Time (min)')
    axes.set_ylabel('CD')
    axes.grid(True)
    
#     ------------------------------------------------------------------    
#       Mass
#     ------------------------------------------------------------------
    plt.figure("Mass")
    axes = plt.gca()    
    for i in range(len(results.segments)):     
        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        energy = results.segments[i].conditions.weights.total_mass[:,0] / Units['kg']
        axes.plot(time, energy, 'o-', color = colors[0], linewidth = thickness)
    plot_format(fig, axes, xlabel = 'Time [mins]',              xlabel_size = xlabel_size,\
                           ylabel = 'Mass [kg]',             ylabel_size = ylabel_size,\
                           title  = 'Aircraft Mass',           title_size  = title_size,\
                           tick_size = tick_size,               tick_rotation = tick_rotation,\
                           grid = grid,                         minor_ticks = minor_ticks) 

    
    # ------------------------------------------------------------------    
    #   Throttle
    # ------------------------------------------------------------------
    fig = plt.figure("Throttle History")
    axes = fig.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        eta  = results.segments[i].conditions.propulsion.throttle[:,0]
        axes.plot(time, eta, 'o-', color = colors[0], linewidth = thickness)
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Throttle')
    axes.set_title('Power Throttle', fontsize = 16)
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
    plt.ylim((0,1.05))
    plot_format(fig, axes, xlabel = 'Time [mins]',              xlabel_size = xlabel_size,\
                           ylabel = 'Throttle [-]',             ylabel_size = ylabel_size,\
                           title  = 'Power Throttle',           title_size  = title_size,\
                           tick_size = tick_size,               tick_rotation = tick_rotation,\
                           grid = grid,                         minor_ticks = minor_ticks)  
    
    # ------------------------------------------------------------------    
    #   SFC
    # ------------------------------------------------------------------
    fig = plt.figure("SFC History")
    axes = fig.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        sfc  = results.segments[i].conditions.propulsion.sfc[:,0]
        axes.plot(time, sfc/Units['lb/hp/h'], 'o-', color = colors[0], linewidth = thickness)
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
#    plt.ylim((0,1.05))
    plot_format(fig, axes, xlabel = 'Time [mins]',              xlabel_size = xlabel_size,\
                           ylabel = 'SFC [lb/hp/h]',             ylabel_size = ylabel_size,\
                           title  = 'Specific Power Consumption',           title_size  = title_size,\
                           tick_size = tick_size,               tick_rotation = tick_rotation,\
                           grid = grid,                         minor_ticks = minor_ticks) 
    
    # ------------------------------------------------------------------    
    #   Fuel FLow
    # ------------------------------------------------------------------
    fig = plt.figure("FF History")
    axes = fig.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        ff  = results.segments[i].conditions.propulsion.fuel_flow[:,0]
        axes.plot(time, ff/Units['lb/h'], 'o-', color = colors[0], linewidth = thickness)
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
#    plt.ylim((0,1.05))
    plot_format(fig, axes, xlabel = 'Time [mins]',              xlabel_size = xlabel_size,\
                           ylabel = 'Fuel FLow [lb/h]',             ylabel_size = ylabel_size,\
                           title  = 'Fuel Flow',           title_size  = title_size,\
                           tick_size = tick_size,               tick_rotation = tick_rotation,\
                           grid = grid,                         minor_ticks = minor_ticks) 
    
    # ------------------------------------------------------------------    
    #   Propeller RPM
    # ------------------------------------------------------------------
#    plt.figure("Propeller Data")
#    axes = plt.gca()    
#    for i in range(len(results.segments)):     
#        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
#        rpm = results.segments[i].conditions.propulsion.propeller_omega[:,0] / Units.rpm
#        axes.plot(time, rpm, 'o-', color = colors[0], linewidth = thickness)
#        axes.tick_params(axis='y', labelcolor=colors[0])
#    axes.set_xlabel('Time [mins]', fontsize = xlabel_size)
#    axes.set_ylabel('RPM', color = colors[0], fontsize = ylabel_size)
#    axes.tick_params(axis='y', labelcolor=colors[0], size = tick_size)
#    axes.set_title('Propeller Data', fontsize = title_size)
#    axes.get_yaxis().get_major_formatter().set_scientific(False)
#    axes.get_yaxis().get_major_formatter().set_useOffset(False) 
#    plt.minorticks_on()
#    axes.grid(True, which = 'both')
#    
#    axes2 = axes.twinx()
#    for i in range(len(results.segments)):     
#        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
#        Q = results.segments[i].conditions.propulsion.propeller_torque[:,0] / Units['N*m']
#        axes2.plot(time, Q, 'o-', color = colors[1], linewidth = thickness)
#        axes2.set_ylabel('Torque [Nm]', color = colors[1], fontsize = ylabel_size)
#        axes2.tick_params(axis='y', labelcolor= colors[1], size = tick_size)
        
   
    
#     ------------------------------------------------------------------
#     Mission Profile
#     ------------------------------------------------------------------
    fig = plt.figure("Mission Profile")
    for segment in results.segments.values():

        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        speed    = segment.conditions.freestream.velocity[:,0] / Units['kt']
        altitude = segment.conditions.freestream.altitude[:,0] / Units.ft

        axes = fig.add_subplot(2,1,1)
        axes.plot( time , altitude , 'o-', color = colors[0], linewidth = thickness )
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        plot_format(fig, axes, xlabel = '',              xlabel_size = xlabel_size,\
                           ylabel = 'Altitude [ft]',             ylabel_size = ylabel_size,\
                           title  = '',           title_size  = title_size,\
                           tick_size = tick_size,               tick_rotation = tick_rotation,\
                           grid = grid,                         minor_ticks = minor_ticks) 
#        axes.set_ylim(0,300)
#        axes.set_ylim(0,5000)

        axes = fig.add_subplot(2,1,2)
        axes.plot( time , speed, 'o-', color = colors[0], linewidth = thickness )
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        plot_format(fig, axes, xlabel = 'Time [mins]',              xlabel_size = xlabel_size,\
                           ylabel = 'Speed [KTAS]',             ylabel_size = ylabel_size,\
                           title  = '',           title_size  = title_size,\
                           tick_size = tick_size,               tick_rotation = tick_rotation,\
                           grid = grid,                         minor_ticks = minor_ticks)
#        axes.set_ylim(0,220)
        
        fig.suptitle('Mission Profile', fontsize = title_size)
    
    return  

def polar(Cl, a, b):
      return a*Cl**2 + b
      

if __name__ == '__main__':
    
    # Define plot formatting    
    title_size = 20
    xlabel_size = 16
    ylabel_size = 16
    tick_size = 14
    tick_rotation = 90
    minor_ticks = True
    grid = True
    
#    colors = ['mediumblue','darkolivegreen','sienna','purple','orange','crimson']
    colors = ['royalblue','indianred','mediumorchid','yellowgreen','orange','orangered']
#    colors = ['b','r','m','g','y','k']
#    colors = ['deepskyblue','orangered','deeppink','limegreen','coral','darkgray']
    thickness = 3 
    
    # Run the analysis
    breakdown, results, vehicle = main()
    

    # Find Average Propeller Efficiency
    segment_time = np.array(range(len(results.segments.values())), float)
    segment_etap = np.array(range(len(results.segments.values())), float)
    for i in range(len(results.segments.values())):
        time = results.segments[i].conditions.frames.inertial.time[:,0]
        etap = results.segments[i].conditions.propulsion.etap[:,0]
        segment_time[i] = time[-1] - time[0]
        segment_etap[i] = np.average(etap)
    
    avgetap = np.average(segment_etap, weights = segment_time)
    

    
    # DRAG POLAR AND DRAG BREAKDOWN
    Cl = np.empty(0)
    Cd = np.empty(0)
    Cdi = np.empty(0)
    Cdp = np.empty(0)
    
    Cd_wing = np.empty(0)
    Cd_flap = np.empty(0)
    Cd_hstab = np.empty(0)
    Cd_vstab = np.empty(0)
    Cd_fus = np.empty(0)
    Cd_mlg = np.empty(0)
    Cd_nlg = np.empty(0)
    Cd_exc = np.empty(0)
    Cd_para = np.empty(0)
    # Put every drag component into a single array for all segments
    for i in range(len(results.segments.values())):
        coeffs = results.segments[i].conditions.aerodynamics
        Cl = np.append(Cl,coeffs.lift_coefficient)
        Cdi = np.append(Cdi,coeffs.drag_breakdown.induced.total)
        Cdp = np.append(Cdp,coeffs.drag_breakdown.parasite.total)
        Cd  = np.append(Cd,coeffs.drag_breakdown.total)
        Cd_wing = np.append(Cd_wing, coeffs.drag_breakdown.parasite.main_wing.parasite_drag_coefficient)
        Cd_flap = np.append(Cd_flap, coeffs.drag_breakdown.parasite.main_wing.flap_drag)
        Cd_hstab = np.append(Cd_hstab, coeffs.drag_breakdown.parasite.horizontal_stabilizer.parasite_drag_coefficient)
        Cd_vstab= np.append(Cd_vstab, coeffs.drag_breakdown.parasite.vertical_stabilizer.parasite_drag_coefficient)
        Cd_fus = np.append(Cd_fus, coeffs.drag_breakdown.parasite.fuselage.parasite_drag_coefficient)
        Cd_mlg = np.append(Cd_mlg, coeffs.drag_breakdown.parasite.main_ldg.parasite_drag_coefficient)
        Cd_nlg = np.append(Cd_nlg, coeffs.drag_breakdown.parasite.nose_ldg.parasite_drag_coefficient)
        Cd_exc = np.append(Cd_exc, coeffs.drag_breakdown.miscellaneous.total)
        Cd_para = np.append(Cd_para, coeffs.drag_breakdown.parasite.total)
    
    # Get the average for each component
    Cd_wing = np.mean(Cd_wing)
    Cd_flap = np.mean(Cd_flap)
    Cd_hstab = np.mean(Cd_hstab)
    Cd_vstab = np.mean(Cd_vstab)
    Cd_fus = np.mean(Cd_fus)
    Cd_mlg = np.mean(Cd_mlg)
    Cd_nlg = np.mean(Cd_nlg)
    Cd_exc = np.mean(Cd_exc)
    Cd_para = np.mean(Cd_para)
    
    # get bkd for component ref area
    parasite_bkd    = results.segments[0].conditions.aerodynamics.drag_breakdown.parasite
    induced_bkd     = results.segments[0].conditions.aerodynamics.drag_breakdown.induced
    
    ## Cruise Induced Drag
    try:
        Cdi_cz = np.mean(results.segments['cruise1'].conditions.aerodynamics.drag_breakdown.induced.total)
        Cdp_cz = np.mean(results.segments['cruise1'].conditions.aerodynamics.drag_breakdown.parasite.total)
    except:
        Cdi_cz = 0.
        print "Cruise Induced Drag not calculated"
    
    # Fit for a 2 term polar
    coeffs, dummy = curve_fit(polar,Cl,Cd)
    polar_a = coeffs[0]
    polar_b = coeffs[1]
    
    orig_cd0    = 0.03689
    orig_k      = 0.04606
    sorted_Cl   = np.sort(Cl)
    orig_polar  = orig_cd0 + orig_k*sorted_Cl**2
    
    
    fig = plt.figure('Drag Polar2')
    axes = fig.gca()
#    axes.plot(Cl,Cdi, ls = '', marker = 'o', color = 'g', label = r'$C_{D_i}$')
#    axes.plot(Cl,Cdp, ls = '', marker = 'o', color = 'b', label = r'$C_{D_p}$')
#    axes.plot(Cl,Cd, ls = '', marker = 'o', color = 'k', label = r'$C_{D}$')
    axes.plot(Cl,Cdi, ls = '', marker = 'o', color = 'g', label = 'Induced Drag Coeff.')
    axes.plot(Cl,Cdp, ls = '', marker = 'o', color = 'b', label = 'Parasite Drag Coeff.')
    axes.plot(Cl,Cd, ls = '', marker = 'o', color = 'k', label = 'Total Drag Coeff.')
    axes.plot(sorted_Cl, orig_polar, ls = '-', marker = '', color = 'r', label = 'Original Polar')
    plot_format(fig, axes, xlabel = r'$C_L$', ylabel = r'$C_D$', title = 'Induced and Parasite Drag Polars')
    axes.legend(fontsize = 'large')
    
    ##FUEL COMSUMPTION  
    try:
        takeoff_fuel = results.segments.takeoff1.conditions.weights.total_mass[0,0] - results.segments.takeoff3.conditions.weights.total_mass[-1,0]
    except:
        print 'Takeoff Fuel not calculated'
        takeoff_fuel = 0.   
    try:
        climb_fuel = results.segments.climb1.conditions.weights.total_mass[0,0] - results.segments.climb5.conditions.weights.total_mass[-1,0]
    except:
        print 'Climb Fuel not calculated'
        climb_fuel = 0.
    try:
        cruise_ff       = np.mean(results.segments.cruise1.conditions.propulsion.fuel_flow)
        cruise_fuel     = results.segments['cruise1'].conditions.weights.total_mass[0,0] - results.segments['cruise1'].conditions.weights.total_mass[-1,0]
    except:
        print 'Cruise Fuel not calculated'
        cruise_ff       = 0.
        cruise_fuel     = 0.
    try:
        descent_fuel = results.segments.descent1.conditions.weights.total_mass[0,0] - results.segments.descent1.conditions.weights.total_mass[-1,0]
    except:
        print 'Descent Fuel not calculated'
        descent_fuel = 0.
    try:
        landing_fuel = results.segments.approach1.conditions.weights.total_mass[0,0] - results.segments.touchdown.conditions.weights.total_mass[-1,0]
    except:
        print 'Landing Fuel not calculated'
        landing_fuel = 0.
    try:
        reserve_fuel = results.segments.reserve_climb1.conditions.weights.total_mass[0,0] - results.segments.touchdown.conditions.weights.total_mass[-1,0]
    except:
        print 'Reserve Fuel not calculated'
        reserve_fuel = 0.
    
    total_fuel = vehicle.mass_properties.takeoff - results.segments[-1].conditions.weights.total_mass[-1,0]
    
    
    ## RANGES ##
    full_range = results.segments[-1].conditions.frames.inertial.position_vector[-1,0] - results.segments[0].conditions.frames.inertial.position_vector[0,0]
    try:
        real_range      = results.segments['descent1'].conditions.frames.inertial.position_vector[-1,0] - results.segments['climb1'].conditions.frames.inertial.position_vector[0,0]
    except:
        print "Real Range not calculated"
        real_range = 0.

        
    
        
        
    ## OUTPUT DATA
    
    print ' ','*'* 74, '\n{:^76s}\n'.format('CONVENTIONAL CARAVAN'),' ','*'* 74   
    print ' ','*'* 74, '\n{:^76s}\n'.format('General'),' ','*'* 74  
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Ramp Weight",vehicle.mass_properties.ramp,"kg")
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Takeoff Weight",vehicle.mass_properties.takeoff,"kg")
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Payload",vehicle.weight_breakdown.payload,"kg")
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Fuel",vehicle.weight_breakdown.fuel,"kg")
    print ' ','*'* 74, '\n{:^76s}\n'.format('Weights'),' ','*'* 74  
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Wing Weight",vehicle.weight_breakdown.main_wing,"kg")
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Prop Sys Weight",vehicle.weight_breakdown.propulsive_system,"kg")
    print ""
    print "\t {:>20}\t {:>6.1f}\t {:>5}\t {:>6.1f}\t {:>5}".format("Engine",vehicle.propulsors.network.engine.rated_power/Units['kW'],'kW',vehicle.propulsors.network.engine.mass_properties.mass,"kg")
    print "\t {:>20}\t {:>6.1}\t {:>5}\t {:>6.1f}\t {:>5}".format("Propeller",'','',vehicle.propulsors.network.propeller.mass_properties.mass,"kg")
    print ' ','*'* 74, '\n{:^76s}\n'.format('Drag Coefficients'),' ','*'* 74 
    print "\t Drag Polar: {:>6.5f} + {:>6.5f} Cl^2".format(polar_b, polar_a)
    print "\t Oswald: {:>6.5f}, AR:{:>6.5f}, k:{:>6.5f}".format(np.mean(induced_bkd.efficiency_factor), induced_bkd.aspect_ratio, np.mean(induced_bkd.k_factor))
    print "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.3f}\t {:>5}".format("Parasite Total",Cd_para,'Cdp',vehicle.reference_area,"m2")
    print "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.3f}\t {:>5}".format("Induced (avg)",np.mean(Cdi),'Cdi',vehicle.reference_area,"m2")
    print "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.3f}\t {:>5}".format("Cruise Induced (avg)",Cdi_cz,'Cdi',vehicle.reference_area,"m2")
    print ""
    print "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}".format("Main Wing",Cd_wing,'Cdp',parasite_bkd.main_wing.reference_area,"m2")
    print "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}".format("Flaps",Cd_flap,'Cdp',parasite_bkd.main_wing.reference_area,"m2")
    print "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}".format("Vertical Stab",Cd_vstab,'Cdp',parasite_bkd.vertical_stabilizer.reference_area,"m2")
    print "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}".format("Horizontal Stab",Cd_hstab,'Cdp',parasite_bkd.horizontal_stabilizer.reference_area,"m2")
    print "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}".format("Fuselage",Cd_fus,'Cdp',parasite_bkd.fuselage.reference_area,"m2")
    print "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}".format("Main Ldg",Cd_mlg,'Cdp',parasite_bkd.main_ldg.reference_area,"m2")
    print "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}".format("Nose Ldg",Cd_nlg,'Cdp',parasite_bkd.nose_ldg.reference_area,"m2")
    print "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}".format("Excrescences",Cd_exc,'Cdp',results.segments[0].conditions.aerodynamics.drag_breakdown.miscellaneous.reference_area,"m2")
    print "\n"
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Empty Weight",breakdown.empty,"kg")
    print "\n"
    print ' ','*'* 74, '\n{:^76s}\n'.format('Propulsive Analysis'),' ','*'* 74 
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Takeoff Fuel",takeoff_fuel,"kg")
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Climb Fuel",climb_fuel,"kg")
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Cruise Fuel",cruise_fuel,"kg")
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Descent Fuel",descent_fuel,"kg")
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Landing Fuel",landing_fuel,"kg")
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Reserve Fuel",reserve_fuel,"kg")
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Cruise Fuel Flow",cruise_ff/Units['lb/h'],"lb/h")
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Mission Fuel",total_fuel,"kg")
    print "\n"
    print ' ','*'* 74, '\n{:^76s}\n'.format('Range Analysis'),' ','*'* 74 
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Full Range",full_range/Units['nmi'],"nmi")
    print "\t {:>20}\t {:>6.1f}\t {:>5}".format("Real Range",real_range/Units['nmi'],"nmi")


## PRINT TO FILE
    if True:
        f = open('Conv_Caravan_Summary_MAIN{:3.0f}nmi_RESERVE{:3.0f}nmi.txt'.format((real_range)/Units['nmi'], (full_range - real_range)/Units['nmi']),'w+')
        f.write('\n{:^76s}\n'.format('CONVENTIONAL CARAVAN'))   
        f.write('\n{:^76s}\n'.format('General'))  
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Ramp Weight",vehicle.mass_properties.ramp,"kg"))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Takeoff Weight",vehicle.mass_properties.takeoff,"kg"))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Payload",vehicle.weight_breakdown.payload,"kg"))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Fuel",vehicle.weight_breakdown.fuel,"kg"))
        f.write('\n{:^76s}\n'.format('Weights'))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Wing Weight",vehicle.weight_breakdown.main_wing,"kg"))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Prop Sys Weight",vehicle.weight_breakdown.propulsive_system,"kg"))
        f.write( "\n")
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\t {:>6.1f}\t {:>5}\n".format("Engine",vehicle.propulsors.network.engine.rated_power/Units['kW'],'kW',vehicle.propulsors.network.engine.mass_properties.mass,"kg"))
        f.write( "\t {:>20}\t {:>6.1}\t {:>5}\t {:>6.1f}\t {:>5}\n".format("Propeller",'','',vehicle.propulsors.network.propeller.mass_properties.mass,"kg"))
        f.write( '\n{:^76s}\n'.format('Drag Coefficients'))
        f.write( "\t Drag Polar: {:>6.5f} + {:>6.5f} Cl^2\n".format(polar_b, polar_a))
        f.write( "\t Oswald: {:>6.5f}, AR:{:>6.5f}, k:{:>6.5f}\n".format(np.mean(induced_bkd.efficiency_factor), induced_bkd.aspect_ratio, np.mean(induced_bkd.k_factor)))
        f.write( "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.3f}\t {:>5}\n".format("Parasite Total",Cd_para,'Cdp',vehicle.reference_area,"m2"))
        f.write( "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.3f}\t {:>5}\n".format("Induced (avg)",np.mean(Cdi),'Cdi',vehicle.reference_area,"m2"))
        f.write( "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.3f}\t {:>5}\n".format("Cruise Induced (avg)",Cdi_cz,'Cdi',vehicle.reference_area,"m2"))
        f.write( "\n")
        f.write( "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}\n".format("Main Wing",Cd_wing,'Cdp',parasite_bkd.main_wing.reference_area,"m2"))
        f.write( "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}\n".format("Flaps",Cd_flap,'Cdp',parasite_bkd.main_wing.reference_area,"m2"))
        f.write( "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}\n".format("Vertical Stab",Cd_vstab,'Cdp',parasite_bkd.vertical_stabilizer.reference_area,"m2"))
        f.write( "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}\n".format("Horizontal Stab",Cd_hstab,'Cdp',parasite_bkd.horizontal_stabilizer.reference_area,"m2"))
        f.write( "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}\n".format("Fuselage",Cd_fus,'Cdp',parasite_bkd.fuselage.reference_area,"m2"))
        f.write( "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}\n".format("Main Ldg",Cd_mlg,'Cdp',parasite_bkd.main_ldg.reference_area,"m2"))
        f.write( "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}\n".format("Nose Ldg",Cd_nlg,'Cdp',parasite_bkd.nose_ldg.reference_area,"m2"))
        f.write( "\t {:>20}\t {:>6.5f} {:>5}\t {:>6.1f}\t {:>5}\n".format("Excrescences",Cd_exc,'Cdp',results.segments[0].conditions.aerodynamics.drag_breakdown.miscellaneous.reference_area,"m2"))
        f.write( "\n")
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Empty Weight",breakdown.empty,"kg"))
        f.write( "\n")
        f.write( '\n{:^76s}\n'.format('Propulsive Analysis'))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Takeoff Fuel",takeoff_fuel,"kg"))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Climb Fuel",climb_fuel,"kg"))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Cruise Fuel",cruise_fuel,"kg"))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Descent Fuel",descent_fuel,"kg"))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Landing Fuel",landing_fuel,"kg"))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Reserve Fuel",reserve_fuel,"kg"))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Cruise Fuel Flow",cruise_ff/Units['lb/h'],"lb/h"))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Mission Fuel",total_fuel,"kg"))
        f.write( "\n")
        f.write('\n{:^76s}\n'.format('Range Analysis'))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Full Range",full_range/Units['nmi'],"nmi"))
        f.write( "\t {:>20}\t {:>6.1f}\t {:>5}\n".format("Real Range",real_range/Units['nmi'],"nmi"))
        f.close()
        
        
        
        prop_T = np.empty(0)
        prop_P = np.empty(0)
        prop_omega = np.empty(0)
        prop_etap = np.empty(0)
        engine_P = np.empty(0)
        engine_throttle = np.empty(0)
        engine_sfc = np.empty(0)
        engine_fuelflow = np.empty(0)
        speed = np.empty(0)
        altitude = np.empty(0)
        CL = np.empty(0)
        CD = np.empty(0)
        CL_max_ub = np.empty(0)
        CDi = np.empty(0)
        CDp = np.empty(0)
        time = np.empty(0)
        for i in range(len(results.segments.values())):
            conditions  = results.segments[i].conditions
            prop_T      = np.append(prop_T, conditions.propulsion.thrust[:,0])
            prop_P      = np.append(prop_P, conditions.propulsion.propeller_power[:,0]/Units['kW'])
            prop_omega  = np.append(prop_omega, conditions.propulsion.propeller_omega[:,0]/Units['rpm'])
            prop_etap   = np.append(prop_etap, conditions.propulsion.etap[:,0])
            engine_P        = np.append(engine_P, conditions.propulsion.engine_power[:,0]/Units['kW'])
            engine_throttle = np.append(engine_throttle, conditions.propulsion.throttle[:,0])
            engine_sfc      = np.append(engine_sfc, conditions.propulsion.sfc[:,0]/Units['lb/hp/h'])
            engine_fuelflow = np.append(engine_fuelflow, conditions.propulsion.fuel_flow[:,0]/Units['lb/h'])
            speed       = np.append(speed, conditions.freestream.velocity[:,0]/Units['kts'])
            altitude    = np.append(altitude, -conditions.frames.inertial.position_vector[:,2]/Units['ft'])
            CL          = np.append(CL, conditions.aerodynamics.lift_coefficient[:,0])
            CD          = np.append(CD, conditions.aerodynamics.drag_coefficient[:,0])
            CL_max_ub   = np.append(CL_max_ub, conditions.aerodynamics.unblown_maximum_lift_coefficient[:,0])  
            CDi         = np.append(CDi, conditions.aerodynamics.drag_breakdown.induced.total[:,0])
            CDp         = np.append(CDp, conditions.aerodynamics.drag_breakdown.parasite.total[:,0])
            time        = np.append(time, conditions.frames.inertial.time[:,0])
            
        database = np.stack([prop_T, prop_P, prop_omega, prop_etap, engine_P, engine_throttle, engine_sfc, engine_fuelflow, \
                             speed, altitude, CL, CD, CL_max_ub, CDi, CDp], 1)
        
        outputs_list = ['prop_T', 'prop_P', 'prop_omega', 'prop_etap', 'engine_P', 'throttle', 'sfc', 'fuelflow',\
                        'speed', 'altitude', 'CL', 'CD', 'CL_max_ub', 'CDi', 'CDp']
        f = open('Conv_Caravan_Data_MAIN{:3.0f}nmi_RESERVE{:3.0f}nmi.txt'.format((real_range)/Units['nmi'], (full_range - real_range)/Units['nmi']),'w+')
        for i, column in enumerate(outputs_list):
            f.write('{:<20s}\t'.format(column))
        f.write('\n')
        for i in range(np.size(database, 0)):
            for j in range(np.size(database,1)):
                f.write('{:<20.10f}\t'.format(database[i,j]))
            f.write('\n')
        f.close()
    
    plt.show()