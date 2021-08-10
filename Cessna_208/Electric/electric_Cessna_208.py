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
from SUAVE.Core import Data, Container
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion import propeller_design 
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_energy_and_power, initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv    
from SUAVE.Plots.Geometry_Plots.plot_vehicle import plot_vehicle
from SUAVE.Methods.Geometry.Three_Dimensional.compute_span_location_from_chord_length import compute_span_location_from_chord_length
from SUAVE.Methods.Flight_Dynamics.Static_Stability.Approximations.datcom import datcom
from SUAVE.Methods.Flight_Dynamics.Static_Stability.Approximations.Supporting_Functions.trapezoid_ac_x import trapezoid_ac_x

def vehicle_setup(): 
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------        
    vehicle                                     = SUAVE.Vehicle()
    vehicle.tag                                 = 'Cessna_208'
                                                
    # ------------------------------------------------------------------
    #   Vehicle-level Properties - Revised
    # ------------------------------------------------------------------    

    # Vehicle level mass properties
    # The maximum takeoff gross weight is used by a number of methods, most notably the weight
    # method. However, it does not directly inform mission analysis.
    vehicle.mass_properties.max_takeoff               = 3629. * Units.kilogram 
    # The takeoff weight is used to determine the weight of the vehicle at the start of the mission
    vehicle.mass_properties.takeoff                   = 3645. * Units.kilogram   
    # Operating empty may be used by various weight methods or other methods. Importantly, it does
    # not constrain the mission analysis directly, meaning that the vehicle weight in a mission
    # can drop below this value if more fuel is needed than is available.
    vehicle.mass_properties.operating_empty           = 1832. * Units.kilogram 
    # The maximum zero fuel weight is also used by methods such as weights
    vehicle.mass_properties.max_zero_fuel             = 2351.0 * Units.kilogram
    # Cargo weight typically feeds directly into weights output and does not affect the mission
    vehicle.mass_properties.cargo                     = 372.  * Units.kilogram   
    
    # Envelope properties
    # These values are typical FAR values for a transport of this type
    vehicle.envelope.ultimate_load      = 3.75
    vehicle.envelope.limit_load         = 2.5

    # Vehicle level parameters
    # The vehicle reference area typically matches the main wing reference area 
    vehicle.reference_area              = 25.96 * Units['meters**2']  
    # Number of passengers, control settings, and accessories settings are used by the weights
    # methods
    vehicle.passengers                  = 10.
    vehicle.systems.control             = "fully powered" 
    vehicle.systems.accessories         = "medium range"
    vehicle.max_lift_coefficient_factor = 0.87889


    # ------------------------------------------------------------------        
    #  Landing Gear - Revised
    # ------------------------------------------------------------------ 
    
    # The settings here can be used for noise analysis
    landing_gear = SUAVE.Components.Landing_Gear.Landing_Gear()
    landing_gear.tag = "main_landing_gear"
    
    landing_gear.main_tire_diameter = 0.6 * Units.m
    landing_gear.nose_tire_diameter = 0.5 * Units.m
    # landing_gear.main_strut_length  = 1.8 * Units.m
    # landing_gear.nose_strut_length  = 1.3 * Units.m
    landing_gear.main_units  = 2    # Number of main landing gear
    landing_gear.nose_units  = 1    # Number of nose landing gear
    landing_gear.main_wheels = 1    # Number of wheels on the main landing gear
    landing_gear.nose_wheels = 1    # Number of wheels on the nose landing gear      
    vehicle.landing_gear = landing_gear


    # ------------------------------------------------------------------        
    #   Main Wing - Revised
    # ------------------------------------------------------------------        

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.aspect_ratio            = 9.71 # (wing span² / wing area)
    # Quarter chord sweep is used as the driving sweep in most of the low fidelity analysis methods.
    # If a different known value (such as leading edge sweep) is given, it should be converted to
    # quarter chord sweep and added here. In some cases leading edge sweep will be used directly as
    # well, and can be entered here too.
    wing.sweeps.quarter_chord    = 2.74 * Units.deg # (cotan((root chord - tip chord) / wing span))
    wing.thickness_to_chord      = 0.15 #(wing thickness / chord ratio)
    wing.taper                   = 0.616 # (tip chord / root chord)
    wing.spans.projected         = 15.88 * Units.meter 
    wing.chords.root             = 1.98 * Units.meter 
    wing.chords.tip              = 1.22 * Units.meter 
    wing.chords.mean_aerodynamic = 1.63 * Units.meter # (root chord * 2/3 * ((1 + taper ratio + (taper ratio)²) / (1 + taper ratio))
    wing.areas.reference         = 25.96 * Units['meters**2'] # (((root chord + tip chord) * wing length) / 2)
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees
    wing.vertical                = False 
    wing.symmetric               = True 
    # The high lift flag controls aspects of maximum lift coefficient calculations
    wing.high_lift               = True
    # The dynamic pressure ratio is used in stability calculations
    wing.dynamic_pressure_ratio  = 1.0 

    SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing)

    # ------------------------------------------------------------------
    #   Main Wing Control Surfaces
    # ------------------------------------------------------------------
    
    # Information in this section is used for high lift calculations and when conversion to AVL
    # is desired.
    
    # Deflections will typically be specified separately in individual vehicle configurations.
    

    flap                       = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                   = 'flap' 
    flap.span                  = 4.84
    flap.span_fraction_start   = 1.71 / 2
    flap.span_fraction_end     = flap.span_fraction_start + flap.span  
    flap.deflection            = 0.0 * Units.degrees
    flap.configuration_type    = 'single_slotted'
    flap.chord_fraction        = 0.41   
    wing.append_control_surface(flap) 

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
    
    fuselage.propeller_radius = prop.tip_radius

    # add to vehicle
    vehicle.append_component(fuselage)


    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'
    
    # Number of coach seats is used in some weights methods
    fuselage.number_coach_seats    = vehicle.passengers #
    # The seats abreast can be used along with seat pitch and the number of coach seats to
    # determine the length of the cabin if desired.
    fuselage.seats_abreast         = 2 #
    fuselage.seat_pitch            = 1     * Units.meter #
    # Fineness ratios are used to determine VLM fuselage shape and sections to use in OpenVSP
    # output
    fuselage.fineness.nose         = 1.6
    fuselage.fineness.tail         = 2.
    # Nose and tail lengths are used in the VLM setup
    fuselage.lengths.nose          = 6.4   * Units.meter
    fuselage.lengths.tail          = 8.0   * Units.meter
    fuselage.lengths.total         = 38.02 * Units.meter
    # Fore and aft space are added to the cabin length if the fuselage is sized based on
    # number of seats
    fuselage.lengths.fore_space    = 6.    * Units.meter
    fuselage.lengths.aft_space     = 5.    * Units.meter
    fuselage.width                 = 3.74  * Units.meter
    fuselage.heights.maximum       = 3.74  * Units.meter
    fuselage.effective_diameter    = 3.74     * Units.meter
    fuselage.areas.side_projected  = 142.1948 * Units['meters**2'] 
    fuselage.areas.wetted          = 446.718  * Units['meters**2'] 
    fuselage.areas.front_projected = 12.57    * Units['meters**2'] 
    # Maximum differential pressure between the cabin and the atmosphere
    fuselage.differential_pressure = 5.0e4 * Units.pascal
    
    # Heights at different longitudinal locations are used in stability calculations and
    # in output to OpenVSP
    fuselage.heights.at_quarter_length          = 3.74 * Units.meter
    fuselage.heights.at_three_quarters_length   = 3.65 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 3.74 * Units.meter
    
    # add to vehicle
    vehicle.append_component(fuselage)


    #---------------------------------------------------------------------------------------------
    # DEFINE PROPELLER
    #---------------------------------------------------------------------------------------------
    # build network    
    net = Battery_Propeller() 
    net.number_of_engines       = 2.
    net.nacelle_diameter        = 42 * Units.inches
    net.engine_length           = 0.01 * Units.inches
    net.areas                   = Data()
    net.areas.wetted            = 0.01*(2*np.pi*0.01/2)    


    # Component 1 the ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc        = esc

    # Component 2 the Propeller
    # Design the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller() 

    prop.number_of_blades       = 2.0
    prop.freestream_velocity    = 135.*Units['mph']    
    prop.angular_velocity       = 1300.  * Units.rpm  
    prop.tip_radius             = 76./2. * Units.inches
    prop.hub_radius             = 8.     * Units.inches
    prop.design_Cl              = 0.8
    prop.design_altitude        = 12000. * Units.feet
    prop.design_altitude        = 12000. * Units.feet
    prop.design_thrust          = 1200.  
    prop.origin                 = [[2.,2.5,0.784],[2.,-2.5,0.784]]         
    prop.rotation               = [-1,1] 
    prop.symmetry               = True

    prop.airfoil_geometry       =  ['../Airfoils/NACA_4412.txt'] 
    prop.airfoil_polars         = [['../Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                    '../Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                    '../Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                    '../Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                    '../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]

    prop.airfoil_polar_stations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]       
    prop                        = propeller_design(prop)    
    net.propeller               = prop    
    
    # Component 8 the Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.mass_properties.mass = 500. * Units.kg  
    bat.specific_energy      = 350. * Units.Wh/Units.kg
    bat.resistance           = 0.006
    bat.max_voltage          = 500.
    
    initialize_from_mass(bat,bat.mass_properties.mass)
    net.battery              = bat 
    net.voltage              = bat.max_voltage
    
    # Component 9 Miscellaneous Systems 
    sys = SUAVE.Components.Systems.System()
    sys.mass_properties.mass = 5 # kg
    
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------
    # Propeller  motor
    # Component 4 the Motor
    motor                              = SUAVE.Components.Energy.Converters.Motor()
    etam                               = 0.95
    v                                  = bat.max_voltage *3/4
    omeg                               = prop.angular_velocity  
    io                                 = 4.0 
    start_kv                           = 1
    end_kv                             = 25
    # do optimization to find kv or just do a linspace then remove all negative values, take smallest one use 0.05 change
    # essentially you are sizing the motor for a particular rpm which is sized for a design tip mach 
    # this reduces the bookkeeping errors     
    possible_kv_vals                   = np.linspace(start_kv,end_kv,(end_kv-start_kv)*20 +1 , endpoint = True) * Units.rpm
    res_kv_vals                        = ((v-omeg/possible_kv_vals)*(1.-etam*v*possible_kv_vals/omeg))/io  
    positive_res_vals                  = np.extract(res_kv_vals > 0 ,res_kv_vals) 
    kv_idx                             = np.where(res_kv_vals == min(positive_res_vals))[0][0]   
    kv                                 = possible_kv_vals[kv_idx]  
    res                                = min(positive_res_vals) 

    motor.mass_properties.mass         = 10. * Units.kg
    motor.origin                       = prop.origin  
    motor.propeller_radius             = prop.tip_radius   
    motor.speed_constant               = 0.35 
    motor.resistance                   = res
    motor.no_load_current              = io 
    motor.gear_ratio                   = 1. 
    motor.gearbox_efficiency           = 1. # Gear box efficiency     
    net.motor                          = motor 


    # Component 6 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 10. #Watts 
    payload.mass_properties.mass = 1.0 * Units.kg
    net.payload                  = payload

    # Component 7 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 20. #Watts  
    net.avionics        = avionics      

    vehicle.append_component(net)  
           

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
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.V2_VS_ratio = 1.21
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.Vref_VS_ratio = 1.23
    configs.append(config)   
     
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'    
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.V2_VS_ratio = 1.21
    
    # payload?
    
    configs.append(config)
    
    
    return configs