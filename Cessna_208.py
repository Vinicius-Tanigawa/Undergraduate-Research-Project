# Cessna_208.py


# Created:  Feb 2017, M. Vegh 
# Modified: Feb 2018, M. Vegh 
# Modified: Mar 2020, M. Clarke
# Modified: Jun 2021, V. Tanigawa


# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import numpy as np
import SUAVE
from SUAVE.Core import Units
from SUAVE.Core import (
    Data, Container,
)
from SUAVE.Methods.Geometry.Three_Dimensional.compute_span_location_from_chord_length import compute_span_location_from_chord_length
from SUAVE.Methods.Flight_Dynamics.Static_Stability.Approximations.datcom import datcom
from SUAVE.Methods.Flight_Dynamics.Static_Stability.Approximations.Supporting_Functions.trapezoid_ac_x import trapezoid_ac_x
from SUAVE.Methods.Propulsion import propeller_design

def vehicle_setup(): 
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------        
    vehicle                                     = SUAVE.Vehicle()
    vehicle.tag                                 = 'Cessna_208'
                                                
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    vehicle.mass_properties.max_takeoff         = 8750. * Units.pounds
    vehicle.mass_properties.takeoff             = 8750. * Units.pounds
    vehicle.mass_properties.ramp                = 8785 * Units.pounds
    vehicle.mass_properties.cargo               = 0. 
    vehicle.mass_properties.max_landing         = 8500 * Units.pounds
                                                                     
    vehicle.envelope.ultimate_load              = 3.8
#    vehicle.envelope.limit_load                 = 3.8
                                                
    cruise_speed                                = 184. * Units.kts
    altitude                                    = 10000. * Units.ft
    atmo                                        = SUAVE.Analyses.Atmospheric.US_Standard_1976()
#    freestream                                  = atmo.compute_values (0.)
#    freestream0                                 = atmo.compute_values (altitude)
#    mach_number                                 = (cruise_speed/freestream.speed_of_sound)[0][0] 

#    vehicle.design_dynamic_pressure             = ( .5 *freestream0.density*(cruise_speed*cruise_speed))[0][0]
#    vehicle.design_mach_number                  =  mach_number    

    vehicle.reference_area                      = 25.96     
    vehicle.passengers                          = 11.
    vehicle.systems.control                     = "fully powered"
    vehicle.systems.accessories                 = "medium range"

    vehicle.max_lift_coefficient_factor = 0.87889

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    wing                                        = SUAVE.Components.Wings.Main_Wing()
    wing.tag                                    = 'main_wing'    
    wing.sweeps.quarter_chord                   = 2.93 * Units.deg
    wing.thickness_to_chord                     = 0.15
    wing.areas.reference                        = vehicle.reference_area
    wing.spans.projected                        = 15.875 
    wing.chords.root                            = 2 * wing.areas.reference / (wing.spans.projected * (1 + wing.taper))
    wing.chords.tip                             = wing.chords.root * wing.taper
    wing.chords.mean_aerodynamic                = (wing.chords.root + wing.chords.tip) / 2
    wing.taper                                  = 0.586
    wing.aspect_ratio                           = wing.spans.projected ** 2 / wing.areas.reference
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
#    wing.origin                                 = [[80.* Units.inches,0,0]]
#    wing.aerodynamic_center                     = [22.* Units.inches,0,0]
    wing.vertical                               = False
    wing.symmetric                              = True
    wing.high_lift                              = True 
#    wing.dynamic_pressure_ratio                 = 1.0 
                                          
    # control surfaces -------------------------------------------
    flap                                        = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                                    = 'flap' 
    flap.span_fraction_start                    = 1.71 / 2
    flap.span_fraction_end                      = flap.span_fraction_start + 4.84  
    flap.deflection                             = 0.0 * Units.degrees
    flap.chord_fraction                         = 0.41 
    flap.configuration_type                     = 'single_slotted'    
    wing.append_control_surface(flap)           
                                                
#    slat                                        = SUAVE.Components.Wings.Control_Surfaces.Slat() 
#    slat.tag                                    = 'slat' 
#    slat.span_fraction_start                    = 0.324 
#    slat.span_fraction_end                      = 0.963     
#    slat.deflection                             = 1.0 * Units.deg
#    slat.chord_fraction                         = 0.1      
#    wing.append_control_surface(slat)  
    
#    SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing) 

    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        
                                                
    wing                                        = SUAVE.Components.Wings.Wing()
    wing.tag                                    = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord                   = 2.05 * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.areas.reference                        = 13.22
    wing.spans.projected                        = 6.25
    wing.chords.root                            = 1.31
    wing.chords.tip                             = 0.81
    wing.chords.mean_aerodynamic                = (wing.chords.root + wing.chords.tip)/2 
    wing.taper                                  = 0.62
    wing.aspect_ratio                           = 2.95
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
#    wing.origin                                 = [[246.* Units.inches,0,0]]
#    wing.aerodynamic_center                     = [20.* Units.inches,0,0]
    wing.vertical                               = False
    wing.symmetric                              = True
#    wing.high_lift                              = False 
#    wing.dynamic_pressure_ratio                 = 0.9

    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing                                        = SUAVE.Components.Wings.Wing()
    wing.tag                                    = 'vertical_stabilizer' 
    wing.sweeps.quarter_chord                   = 14.06 * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.areas.reference                        = 4.21
    wing.spans.projected                        = 3.05
    wing.chords.root                            = 2.21
    wing.chords.tip                             = 0.84
    wing.chords.mean_aerodynamic                = (wing.chords.root + wing.chords.tip) / 2 
    wing.taper                                  = 0.38
    wing.aspect_ratio                           = 2.0
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
#    wing.origin                                 = [[237.* Units.inches,0,0]]
#    wing.aerodynamic_center                     = [20.* Units.inches,0,0] 
    wing.vertical                               = True 
    wing.symmetric                              = False
#    wing.t_tail                                 = False 
#    wing.dynamic_pressure_ratio                 = 1.0

    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage'
#    fuselage.number_coach_seats                 = 4.          
#    fuselage.differential_pressure              = 8*Units.psi                    # Maximum differential pressure
    fuselage.width                              = 1.71
    fuselage.heights.maximum                    = 2.10 + 0.68
    fuselage.lengths.total                      = 16.67
#    fuselage.lengths.empennage                  = 161. * Units.inches  
#    fuselage.lengths.cabin                      = 105. * Units.inches
#    fuselage.lengths.structure                  = fuselage.lengths.total-fuselage.lengths.empennage 
#    fuselage.mass_properties.volume             = .4*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.) #try this as approximation
#    fuselage.mass_properties.internal_volume    = .3*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.)
    fuselage.areas.wetted                       = 75.58
#    fuselage.seats_abreast                      = 2.
#    fuselage.fineness.nose                      = 1.6
#    fuselage.fineness.tail                      = 2.
#    fuselage.lengths.nose                       = 60.  * Units.inches
#    fuselage.fineness_ratio                     = fuselage.lengths.total/fuselage.effective_diameter
    fuselage.heights.at_quarter_length          = 2.10 + 0.68 #Testing, not real value
    fuselage.heights.at_three_quarters_length   = 2.10 + 0.68 #Testing, not real value
    fuselage.heights.at_wing_root_quarter_chord = 0.68 #Testing, not real value
    fuselage.areas.front_projected              = fuselage.width* fuselage.heights.maximum
    fuselage.areas.side_projected               = fuselage.lengths.total* fuselage.heights.maximum
    fuselage.effective_diameter                 = np.sqrt(4*fuselage.areas.front_projected/np.pi)

    fuselage.strut = Data()
    fuselage.strut.thickness_to_chord           = 0.12
    fuselage.strut.chord                        = 0.3
    fuselage.strut.length                       = 2.9

#    fuselage.propeller_radius = prop.prop_attributes.tip_radius

    vehicle.append_component(fuselage)
    
    # ------------------------------------------------------------------
    #   Landing gear
    # ------------------------------------------------------------------  

    landing_gear                                = SUAVE.Components.Landing_Gear.Landing_Gear()

    main_gear                                   = SUAVE.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tag = 'main_ldg'
    main_gear.CD              = 0.615
    main_gear.tire_diameter   = 0.6
    main_gear.tire_width      = 0.2
#    main_gear.strut_length                      = 12. * Units.inches

#    vehicle.append_component(main_gear)

    nose_gear                                   = SUAVE.Components.Landing_Gear.Nose_Landing_Gear()
    nose_gear.tag = 'nose_ldg'
    nose_gear.CD              = 1.9
    nose_gear.tire_diameter   = 0.5
    nose_gear.tire_width      = 0.18
#    nose_gear.strut_length                      = 6. * Units.inches

#    vehicle.append_component(nose_gear) 
                                                
    landing_gear.main                           = main_gear
    landing_gear.nose                           = nose_gear
                                                                           
    vehicle.landing_gear                        = landing_gear
                                                
    # ------------------------------------------------------------------
    #   Fuel
    # ------------------------------------------------------------------    

#    fuel                                        = SUAVE.Attributes.Propellants.Aviation_Gasoline()
#    fuel.mass_properties                        = SUAVE.Components.Mass_Properties() 
#    fuel.number_of_tanks                        = 1.
#    fuel.origin                                 = wing.origin
#    fuel.internal_volume                        = fuel.mass_properties.mass/fuel.density
#    fuel.mass_properties.center_of_gravity      = wing.mass_properties.center_of_gravity
#    fuel.mass_properties.mass                   = 319 *Units.lbs
#    vehicle.fuel                                = fuel

    # ------------------------------------------------------------------
    #   Propeller
    # ------------------------------------------------------------------    
    
    # build network
    net                                         = SUAVE.Components.Energy.Networks.Internal_Combustion_Propeller()
    net.tag                                     = 'internal_combustion'
    net.number_of_engines                       = 1.
    net.nacelle_diameter                        = 0.2 * Units.meters
    net.engine_length                           = 0.01 * Units.meters
    net.areas                                   = Data()
    net.areas.wetted                            = 0
    net.rated_power                             = 675.  * Units.hp
    net.rated_speed                             = 1900. * Units.rpm
    net.propeller_speed                         = 1900 * Units['rpm']
    net.thrust_angle                            = 0.
                                                
    # the engine                    
    net.engine                                  = SUAVE.Components.Energy.Converters.Internal_Combustion_Engine()
    net.engine.sea_level_power                  = 675. * Units.horsepower
    net.engine.flat_rate_altitude               = 0.0
    net.engine.rated_speed                      = 1900. * Units.rpm
#    net.engine.inputs.speed                     = 1900. * Units.rpm
    net.engine.torque_limit                     = 1970 * Units['lbf*ft']
#    net.engine.rated_sfc                        = 0.64 * Units['lb/hp/h'] * 1.02
    net.engine.power_specific_fuel_consumption  = 0.64
    
    # the prop
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_of_blades        = 3.0
    prop.freestream_velocity     = 90.   * Units.knots
    prop.angular_velocity        = 1900.  * Units.rpm
    prop.tip_radius              = 53. * Units.inches
    prop.hub_radius              = 0.15     * Units.meters
    prop.design_Cl               = 0.4
    prop.design_altitude         = 5000. * Units.feet
    prop.design_thrust           = None
    prop.design_power            = 600. * Units.horsepower
    prop.activity_factor         = 90
    prop.weight_class            = 0

#    prop.airfoil_geometry        =  ['../Vehicles/Airfoils/NACA_4412.txt'] 
#    prop.airfoil_polars          = [['../Vehicles//Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
#                                     '../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
#                                     '../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
#                                     '../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
#                                     '../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]

#    prop.airfoil_polar_stations  = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]   
   
    prop                         = propeller_design(prop)   
    
    net.propeller = prop
     
    vehicle.append_component(net) 

    #find uninstalled avionics weight
    Wuav                                        = 2. * Units.lbs
    avionics                                    = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.mass_properties.uninstalled        = Wuav
    vehicle.avionics                            = avionics     

    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle


# ---------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------
  
def configs_setup(vehicle):

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs                                                    = SUAVE.Components.Configs.Config.Container() 
    base_config                                                = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag                                            = 'base'
    configs.append(base_config)
    
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------ 
    config                                                     = SUAVE.Components.Configs.Config(base_config)
    config.tag                                                 = 'cruise'
    config.wings.main_wing.flaps.angle                         = 00 * Units['degree']
    configs.append(config)
    
    
    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------ 
    config                                                     = SUAVE.Components.Configs.Config(base_config)
    config.tag                                                 = 'takeoff' 
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.propulsors.network.propeller_speed                  = 1900 * Units['rpm']
#    config.V2_VS_ratio                                         = 1.21
#    config.maximum_lift_coefficient                            = 2.
    
    configs.append(config)


    # ------------------------------------------------------------------
    #   Approach Configurations
    # ------------------------------------------------------------------
    
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag                                                 = 'approach'
    config.wings.main_wing.flaps.angle                         = 30 * Units['degree']
    config.propulsors.network.propeller_speed                  = 1750 * Units['rpm']
    configs.append(config)
    
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config                                                     = SUAVE.Components.Configs.Config(base_config)
    config.tag                                                 = 'landing' 
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.Vref_VS_ratio                                       = 1.23
    config.maximum_lift_coefficient                            = 2.
                                                               
    configs.append(config)


    # ------------------------------------------------------------------
    #   Stall Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag                                                 = 'stall'
    config.wings.main_wing.flaps.angle                         = 30 * Units['degree']
    config.propulsors.network.propeller_speed                  = 1900 * Units['rpm']    
    configs.append(config)
    
    
    return configs