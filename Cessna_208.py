# Cessna_208.py
#
# Created:  Feb 2017, M. Vegh 
# Modified: Feb 2018, M. Vegh 
# Modified: Mar 2020, M. Clarke 
# Modified: Apr 2021, V. Tanigawa 

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
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_energy_and_power, initialize_from_mass
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller

def vehicle_setup(): 
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------        
    vehicle                                     = SUAVE.Vehicle()
    vehicle.tag                                 = 'Cessna_208_Caravan'
                                                
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff         = 8750. * Units.pounds #modified
    vehicle.mass_properties.takeoff             = 8750. * Units.pounds #modified
    vehicle.mass_properties.max_zero_fuel       = 8750. * Units.pounds #modified
    vehicle.mass_properties.cargo               = 0. 
                                               
    # envelope properties                       
    vehicle.envelope.ultimate_load              = 4.68 #modified
    vehicle.envelope.limit_load                 = 3.8 #modified
                                                
    cruise_speed                                = 184. * Units.kts #modified
    altitude                                    = 10000. * Units.ft #modified
    atmo                                        = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    freestream                                  = atmo.compute_values (0.)
    freestream0                                 = atmo.compute_values (altitude)
    mach_number                                 = (cruise_speed/freestream.speed_of_sound)[0][0] 
    vehicle.design_dynamic_pressure             = ( .5 *freestream0.density*(cruise_speed*cruise_speed))[0][0]
    vehicle.design_mach_number                  =  mach_number
                                                
    # basic parameters                          
    vehicle.reference_area                      = 279. * Units.feet**2 #modified    
    vehicle.passengers                          = 14 #modified

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    wing                                        = SUAVE.Components.Wings.Main_Wing()
    wing.tag                                    = 'main_wing'    
    wing.sweeps.quarter_chord                   = 0.0 * Units.deg
    wing.thickness_to_chord                     = 0.12 
    wing.areas.reference                        = 279. * Units.feet**2 #modified
    wing.spans.projected                        = 52.  * Units.feet + 1. * Units.inches #modified
    wing.chords.root                            = 66. * Units.inches
    wing.chords.tip                             = 45. * Units.inches
    wing.chords.mean_aerodynamic                = 58. * Units.inches
    wing.taper                                  = wing.chords.root/wing.chords.tip
    wing.aspect_ratio                           = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                            = 3.0 * Units.degrees
    wing.twists.tip                             = 1.5 * Units.degrees
    wing.origin                                 = [[80.* Units.inches,0,0]]
    wing.aerodynamic_center                     = [22.* Units.inches,0,0]
    wing.vertical                               = False
    wing.symmetric                              = True
    wing.high_lift                              = True 
    wing.dynamic_pressure_ratio                 = 1.0 
                                          
    # control surfaces -------------------------------------------
    flap                                        = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                                    = 'flap' 
    flap.span_fraction_start                    = 0.15 
    flap.span_fraction_end                      = 0.324    
    flap.deflection                             = 1.0 * Units.deg
    flap.chord_fraction                         = 0.19    
    wing.append_control_surface(flap)           
                                                
    slat                                        = SUAVE.Components.Wings.Control_Surfaces.Slat() 
    slat.tag                                    = 'slat' 
    slat.span_fraction_start                    = 0.324 
    slat.span_fraction_end                      = 0.963     
    slat.deflection                             = 1.0 * Units.deg
    slat.chord_fraction                         = 0.1      
    wing.append_control_surface(slat)  
    
    SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing) 

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        
                                                
    wing                                        = SUAVE.Components.Wings.Wing()
    wing.tag                                    = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord                   = 0.0 * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.areas.reference                        = 5800. * Units.inches**2
    wing.spans.projected                        = 246.  * Units.inches #modified
    wing.chords.root                            = 55. * Units.inches
    wing.chords.tip                             = 30. * Units.inches
    wing.chords.mean_aerodynamic                = 43. * Units.inches 
    wing.taper                                  = wing.chords.root/wing.chords.tip
    wing.aspect_ratio                           = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
    wing.origin                                 = [[246.* Units.inches,0,0]]
    wing.aerodynamic_center                     = [20.* Units.inches,0,0]
    wing.vertical                               = False
    wing.symmetric                              = True
    wing.high_lift                              = False 
    wing.dynamic_pressure_ratio                 = 0.9
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing                                        = SUAVE.Components.Wings.Wing()
    wing.tag                                    = 'vertical_stabilizer' 
    wing.sweeps.quarter_chord                   = 25. * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.areas.reference                        = 3500. * Units.inches**2
    wing.spans.projected                        = 73.   * Units.inches
    wing.chords.root                            = 66. * Units.inches
    wing.chords.tip                             = 27. * Units.inches
    wing.chords.mean_aerodynamic                = 48. * Units.inches 
    wing.taper                                  = wing.chords.root/wing.chords.tip
    wing.aspect_ratio                           = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
    wing.origin                                 = [[237.* Units.inches,0,0]]
    wing.aerodynamic_center                     = [20.* Units.inches,0,0] 
    wing.vertical                               = True 
    wing.symmetric                              = False
    wing.t_tail                                 = False 
    wing.dynamic_pressure_ratio                 = 1.0

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage'
    fuselage.number_coach_seats                 = 4.       
    fuselage.tag                                = 'fuselage'    
    fuselage.differential_pressure              = 8*Units.psi                                 # Maximum differential pressure
    fuselage.width                              = 42.         * Units.inches                  # Width of the fuselage
    fuselage.heights.maximum                    = 62. * Units.inches                          # Height of the fuselage
    fuselage.lengths.total                      = 499.         * Units.inches #Modified       # Length of the fuselage 
    fuselage.lengths.empennage                  = 161. * Units.inches  
    fuselage.lengths.cabin                      = 105. * Units.inches
    fuselage.lengths.structure                  = fuselage.lengths.total-fuselage.lengths.empennage 
    fuselage.mass_properties.volume             = .4*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.) #try this as approximation
    fuselage.mass_properties.internal_volume    = .3*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.)
    fuselage.areas.wetted                       = 30000. * Units.inches**2.
    fuselage.seats_abreast                      = 2.
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2.
    fuselage.lengths.nose                       = 60.  * Units.inches
    fuselage.heights.at_quarter_length          = 62. * Units.inches
    fuselage.heights.at_three_quarters_length   = 62. * Units.inches
    fuselage.heights.at_wing_root_quarter_chord = 23. * Units.inches
    fuselage.areas.front_projected              = fuselage.width* fuselage.heights.maximum
    fuselage.effective_diameter                 = 50. * Units.inches

    # add to vehicle
    vehicle.append_component(fuselage)
    
    # ------------------------------------------------------------------
    #   Landing gear
    # ------------------------------------------------------------------  
    landing_gear                                = SUAVE.Components.Landing_Gear.Landing_Gear()
    main_gear                                   = SUAVE.Components.Landing_Gear.Main_Landing_Gear()
    nose_gear                                   = SUAVE.Components.Landing_Gear.Nose_Landing_Gear()
    main_gear.strut_length                      = 12. * Units.inches #guess based on picture
    nose_gear.strut_length                      = 6. * Units.inches 
                                                
    landing_gear.main                           = main_gear
    landing_gear.nose                           = nose_gear
                                                
    #add to vehicle                             
    vehicle.landing_gear                        = landing_gear
                                                
    #------------------------------------------------------------------
    # Propulsor
    #------------------------------------------------------------------

    # Build Network    
    net = Battery_Propeller() 
    net.number_of_engines       = 1. #modified
    net.nacelle_diameter        = 8.17 * Units.inches #modified
    net.engine_length           = 9.25 * Units.inches #modified
    net.areas                   = Data()
    net.areas.wetted            = 0.01*(2*np.pi*0.01/2)    


    # Component 1 the ESC (MagniDrive)
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.989 #modified 
    esc.mass_properties.mass = 12.0 * Units.kg #modified
    net.esc        = esc
    
    # Component 2 the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_of_blades        = 3.0 #modified
    prop.freestream_velocity     = 119.   * Units.knots
    prop.angular_velocity        = 2650.  * Units.rpm
    prop.tip_radius              = 105. * Units.inches #modified
    prop.hub_radius              = 8.     * Units.inches
    prop.design_Cl               = 0.8
    prop.design_altitude         = 12000. * Units.feet
    prop.design_power            = .64 * 180. * Units.horsepower
    prop.airfoil_geometry        =  ['../Vehicles/NACA_4412.txt'] 
    prop.airfoil_polars          = [['../Vehicles/NACA_4412_polar_Re_50000.txt' ,'../Vehicles/NACA_4412_polar_Re_100000.txt' ,'../Vehicles/NACA_4412_polar_Re_200000.txt' ,
                                     '../Vehicles/NACA_4412_polar_Re_500000.txt' ,'../Vehicles/NACA_4412_polar_Re_1000000.txt' ]]
    prop.airfoil_polar_stations  = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]      
    prop                         = propeller_design(prop)   
    
    net.propeller = prop

    # Component 3 the Motor (HPDM-250)
    motor                      = SUAVE.Components.Energy.Converters.Motor()
    motor.resistance           = 0.008
    motor.no_load_current      = 4.5  * Units.ampere 
    motor.speed_constant       = 5000. * Units['rpm'] #modified
    motor.propeller_radius     = prop.tip_radius
    motor.propeller_Cp         = prop.design_power_coefficient
    motor.gear_ratio           = 4. #modified
    motor.gearbox_efficiency   = .92 #modified
    motor.expected_current     = 250. #modified
    motor.mass_properties.mass = 18.0  * Units.kg #modified
    net.motor                  = motor  

    # Component 4 the Payload
    payload                      = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 50. * Units.watts 
    payload.mass_properties.mass = 17.0 * Units.kg #modified
    net.payload                  = payload
    
    # Component 5 the Avionics
    avionics            = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 50. * Units.watts
    net.avionics        = avionics    
     
    # Component 6 the Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.mass_properties.mass = 500. * Units.kg  
    bat.specific_energy      = 350. * Units.Wh/Units.kg
    bat.resistance           = 0.006
    bat.max_voltage          = 500.
    
    initialize_from_mass(bat,bat.mass_properties.mass)
    net.battery              = bat 
    net.voltage              = bat.max_voltage  

    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle
  
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
    configs.append(config)
    
    
    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------ 
    config                                                     = SUAVE.Components.Configs.Config(base_config)
    config.tag                                                 = 'takeoff' 
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.V2_VS_ratio                                         = 1.21
    config.maximum_lift_coefficient                            = 2.
    
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
    
    
    # done!
    return configs