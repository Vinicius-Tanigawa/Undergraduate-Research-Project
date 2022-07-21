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
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass   
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_optimal_motor
from SUAVE.Methods.Geometry.Three_Dimensional.compute_span_location_from_chord_length import compute_span_location_from_chord_length
from SUAVE.Methods.Flight_Dynamics.Static_Stability.Approximations.datcom import datcom
from SUAVE.Methods.Flight_Dynamics.Static_Stability.Approximations.Supporting_Functions.trapezoid_ac_x import trapezoid_ac_x
from copy import deepcopy

def vehicle_setup(): 
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------        
    vehicle                                     = SUAVE.Vehicle()
    vehicle.tag                                 = 'electric_Cessna_208'
                                                
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # Vehicle level Mass Properties -------------------------------------------
    vehicle.mass_properties.max_takeoff               = 8785. * Units.pound
    vehicle.mass_properties.takeoff                   = 4000. * Units.pound
    vehicle.mass_properties.operating_empty           = 4680. * Units.pound 
    vehicle.mass_properties.cargo                     = 4105.  * Units.pound  
    vehicle.mass_properties.center_of_gravity         = [[4.4634, 0., 0.]] # (Considering CG%_c = 28%)
    # vehicle.mass_properties.moments_of_inertia.tensor = [[3173074.17, 0 , 28752.77565],[0 , 3019041.443, 0],[0, 0, 5730017.433]]
    
    vehicle.design_mach_number                        = 0.289
    vehicle.design_range                              = 1070 * Units.nmi
    vehicle.design_cruise_alt                         = 25000.0 * Units.ft

    # Envelope Properties -------------------------------------------
    vehicle.envelope.ultimate_load      = 3.75
    vehicle.envelope.limit_load         = 2.5

    # Vehicle Level Parameters -------------------------------------------
    vehicle.reference_area              = 25.96 * Units['meters**2']  
    vehicle.passengers                  = 10.
    vehicle.systems.control             = "fully powered" 
    vehicle.systems.accessories         = "medium range"


    # ------------------------------------------------------------------        
    #  Landing Gear
    # ------------------------------------------------------------------ 
    
    landing_gear = SUAVE.Components.Landing_Gear.Landing_Gear()
    landing_gear.tag = "main_landing_gear"
    
    landing_gear.main_tire_diameter = 0.6 * Units.m
    landing_gear.nose_tire_diameter = 0.5 * Units.m
    landing_gear.main_units  = 2
    landing_gear.nose_units  = 1
    landing_gear.main_wheels = 1
    landing_gear.nose_wheels = 1     
    vehicle.landing_gear = landing_gear


    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.aspect_ratio            = 9.71
    wing.sweeps.quarter_chord    = 2.93 * Units.deg
    wing.thickness_to_chord      = 0.195
    wing.taper                   = 0.586

    wing.spans.projected         = 15.875 * Units.meter 

    wing.chords.root             = 2.062 * Units.meter
    wing.chords.tip              = 1.208 * Units.meter 
    wing.chords.mean_aerodynamic = 1.635 * Units.meter

    wing.areas.reference         = 25.96 * Units['meters**2']
    wing.areas.wetted            = 51.82 * Units['meters**2']

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[4.022,0,0.630]]
    wing.aerodynamic_center      = [0,0,0]  

    wing.vertical                = False 
    wing.symmetric               = True 
    wing.high_lift               = True

    wing.number_of_motors        = 0.

    wing.dynamic_pressure_ratio  = 1.0 

    wing.unblown_maximum_lift_coefficient = 1.521365534 

    #   Main Wing Segments -------------------------------------------
    #   Airfoil Geometry File (reference): https://m-selig.ae.illinois.edu/ads/aircraft.html
    #   Airfoil Data Acquired from: http://airfoiltools.com/airfoil/naca5digit?MNaca5DigitForm%5Bcl%5D=0.3&MNaca5DigitForm%5BposKey%5D=15_0&MNaca5DigitForm%5Bthick%5D=15.5&MNaca5DigitForm%5BnumPoints%5D=81&MNaca5DigitForm%5BcosSpace%5D=0&MNaca5DigitForm%5BcosSpace%5D=1&MNaca5DigitForm%5BcloseTe%5D=0&yt0=Plot
    root_airfoil                          = SUAVE.Components.Airfoils.Airfoil()
    root_airfoil.coordinate_file          = '../Airfoils/C208a.txt' #suposition
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'Root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.thickness_to_chord            = 0.195
    segment.dihedral_outboard             = 3. * Units.degrees
    # segment.sweeps.quarter_chord          = 28.225 * Units.degrees
    segment.thickness_to_chord            = 0.195
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment)

    mid_airfoil                           = SUAVE.Components.Airfoils.Airfoil()
    mid_airfoil.coordinate_file           = '../Airfoils/C208b.txt' #suposition
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'Section_2'
    segment.percent_span_location         = 0.5
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.813 #suposition
    segment.thickness_to_chord            = 0.174
    segment.dihedral_outboard             = 3. * Units.degrees
    # segment.sweeps.quarter_chord          = 56.75 * Units.degrees
    segment.thickness_to_chord            = 0.174
    segment.append_airfoil(mid_airfoil)
    wing.append_segment(segment)

    tip_airfoil                           =  SUAVE.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = '../Airfoils/C208c.txt' #suposition
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'Tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0. * Units.degrees
    segment.root_chord_percent            = 0.626 #suposition
    segment.thickness_to_chord            = 0.141
    segment.dihedral_outboard             = 0.
    # segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.141
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)


    #   Main Wing Control Surfaces -------------------------------------------
    flap                       = SUAVE.Components.Wings.Control_Surfaces.Flap()
    flap.tag                   = 'flap'
    flap.span_fraction_start   = 0.855
    flap.span_fraction_end     = 5.695
    flap.deflection            = 0.0 * Units.degrees
    flap.configuration_type    = 'single_slotted'
    flap.chord_fraction        = 0.41
    wing.append_control_surface(flap)

    SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing)

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        


    wing = SUAVE.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'
    
    wing.aspect_ratio            = 2.953
    wing.sweeps.quarter_chord    = 2.05 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.618

    wing.spans.projected         = 6.248 * Units.meter

    wing.chords.root             = 1.31  * Units.meter
    wing.chords.tip              = 0.81 * Units.meter
    wing.chords.mean_aerodynamic = 1.06  * Units.meter

    wing.areas.reference         = 13.22   * Units['meters**2']
    wing.areas.wetted            = 26.44 * Units['meters**2']
    # wing.areas.exposed           = 11.152 * Units['meters**2']
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  

    wing.origin                  = [[10.785,0,0.606]]
    wing.aerodynamic_center      = [0,0,0]

    wing.vertical                = False 
    wing.symmetric               = True

    wing.dynamic_pressure_ratio  = 0.9  

    #   Horizontal Stabilizer Segments -------------------------------------------
    segment                        = SUAVE.Components.Wings.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.dihedral_outboard      = 0. * Units.degrees
    # segment.sweeps.quarter_chord   = 28.2250  * Units.degrees 
    segment.thickness_to_chord     = 0.176
    wing.append_segment(segment)

    segment                        = SUAVE.Components.Wings.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.623 #suposition               
    segment.dihedral_outboard      = 0 * Units.degrees
    # segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = 0.282
    wing.append_segment(segment)

    #   Control Surfaces -------------------------------------------
    elevator                       = SUAVE.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    # elevator.span_fraction_start   = 0.09
    # elevator.span_fraction_end     = 0.92
    elevator.deflection            = 0.0  * Units.deg
    # elevator.chord_fraction        = 0.3
    wing.append_control_surface(elevator)

    #   add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------


    wing = SUAVE.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'    

    wing.aspect_ratio            = 2.0
    wing.sweeps.quarter_chord    = 14.06 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.38

    wing.spans.projected         = 3.05 * Units.meter
    wing.total_length            = wing.spans.projected 

    wing.chords.root             = 2.21  * Units.meter
    wing.chords.tip              = 0.84  * Units.meter
    wing.chords.mean_aerodynamic = 1.525   * Units.meter
    
    wing.areas.reference         = 4.21 * Units['meters**2']
    wing.areas.wetted            = 8.42 * Units['meters**2']
    
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees
    
    wing.origin                  = [[9.046,0,0.95]]
    wing.aerodynamic_center      = [0,0,0]
      
    wing.vertical                = True 
    wing.symmetric               = False
    wing.t_tail                  = False

    wing.dynamic_pressure_ratio  = 1.0

    # Vertical Stabilizer Segments -------------------------------------------
    # segment                               = SUAVE.Components.Wings.Segment()
    # segment.tag                           = 'root'
    # segment.percent_span_location         = 0.0
    # segment.twist                         = 0. * Units.deg
    # segment.root_chord_percent            = 1.
    # segment.dihedral_outboard             = 0 * Units.degrees
    # segment.sweeps.quarter_chord          = 61.485 * Units.degrees  
    # segment.thickness_to_chord            = .1
    # wing.append_segment(segment)

    # segment                               = SUAVE.Components.Wings.Segment()
    # segment.tag                           = 'segment_1'
    # segment.percent_span_location         = 0.2962
    # segment.twist                         = 0. * Units.deg
    # segment.root_chord_percent            = 0.45
    # segment.dihedral_outboard             = 0. * Units.degrees
    # segment.sweeps.quarter_chord          = 31.2 * Units.degrees   
    # segment.thickness_to_chord            = .1
    # wing.append_segment(segment)

    # segment                               = SUAVE.Components.Wings.Segment()
    # segment.tag                           = 'segment_2'
    # segment.percent_span_location         = 1.0
    # segment.twist                         = 0. * Units.deg
    # segment.root_chord_percent            = 0.1183 
    # segment.dihedral_outboard             = 0.0 * Units.degrees
    # segment.sweeps.quarter_chord          = 0.0    
    # segment.thickness_to_chord            = .1  
    # wing.append_segment(segment)

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------


    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'
    
    fuselage.number_coach_seats    = vehicle.passengers
    fuselage.seats_abreast         = 2
    fuselage.seat_pitch            = 1     * Units.meter
    fuselage.fineness.nose         = 2.133
    fuselage.fineness.tail         = 3.104

    fuselage.lengths.nose          = 3.251   * Units.meter
    fuselage.lengths.tail          = 4.529   * Units.meter
    # fuselage.lengths.cabin         = 28.85
    fuselage.lengths.total         = 16.67 * Units.meter
    fuselage.lengths.fore_space    = 0.    * Units.meter
    fuselage.lengths.aft_space     = 0.    * Units.meter

    fuselage.width                 = 1.71  * Units.meter

    fuselage.heights.maximum       = 2.78  * Units.meter
    fuselage.heights.at_quarter_length          = 1.684 * Units.meter
    fuselage.heights.at_three_quarters_length   = 1.643 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 1.684 * Units.meter

    fuselage.areas.side_projected  = 21.353 * Units['meters**2']
    fuselage.areas.wetted          = 75.58  * Units['meters**2']
    fuselage.areas.front_projected = 4.7538    * Units['meters**2']

    fuselage.strut = Data()
    fuselage.strut.thickness_to_chord = 0.12
    fuselage.strut.chord    = 0.3
    fuselage.strut.length   = 2.9 

    fuselage.effective_diameter    = 2.46     * Units.meter

    fuselage.differential_pressure = 0. * Units.pascal

    # Fuselage Segment -------------------------------------------
    # segment                                     = SUAVE.Components.Fuselages.Segment() 
    # segment.tag                                 = 'segment_0'    
    # segment.percent_x_location                  = 0.0000
    # segment.percent_z_location                  = -0.00144 
    # segment.height                              = 0.0100 
    # segment.width                               = 0.0100  
    # fuselage.Segments.append(segment)   
    
    # # Segment  
    # segment                                     = SUAVE.Components.Fuselages.Segment() 
    # segment.tag                                 = 'segment_1'    
    # segment.percent_x_location                  = 0.00576 
    # segment.percent_z_location                  = -0.00144 
    # segment.height                              = 0.7500
    # segment.width                               = 0.6500
    # fuselage.Segments.append(segment)   
    
    # # Segment                                   
    # segment                                     = SUAVE.Components.Fuselages.Segment()
    # segment.tag                                 = 'segment_2'   
    # segment.percent_x_location                  = 0.02017 
    # segment.percent_z_location                  = 0.00000 
    # segment.height                              = 1.52783 
    # segment.width                               = 1.20043 
    # fuselage.Segments.append(segment)      
    
    # # Segment                                   
    # segment                                     = SUAVE.Components.Fuselages.Segment()
    # segment.tag                                 = 'segment_3'   
    # segment.percent_x_location                  = 0.03170 
    # segment.percent_z_location                  = 0.00000 
    # segment.height                              = 1.96435 
    # segment.width                               = 1.52783 
    # fuselage.Segments.append(segment)   

    # # Segment                                   
    # segment                                     = SUAVE.Components.Fuselages.Segment()
    # segment.tag                                 = 'segment_4'   
    # segment.percent_x_location                  = 0.04899 	
    # segment.percent_z_location                  = 0.00431 
    # segment.height                              = 2.72826 
    # segment.width                               = 1.96435 
    # fuselage.Segments.append(segment)   
    
    # # Segment                                   
    # segment                                     = SUAVE.Components.Fuselages.Segment()
    # segment.tag                                 = 'segment_5'   
    # segment.percent_x_location                  = 0.07781 
    # segment.percent_z_location                  = 0.00861 
    # segment.height                              = 3.49217 
    # segment.width                               = 2.61913 
    # fuselage.Segments.append(segment)     
    
    # # Segment                                   
    # segment                                     = SUAVE.Components.Fuselages.Segment()
    # segment.tag                                 = 'segment_6'   
    # segment.percent_x_location                  = 0.10375 
    # segment.percent_z_location                  = 0.01005 
    # segment.height                              = 3.70130 
    # segment.width                               = 3.05565 
    # fuselage.Segments.append(segment)             
     
    # # Segment                                   
    # segment                                     = SUAVE.Components.Fuselages.Segment()
    # segment.tag                                 = 'segment_7'   
    # segment.percent_x_location                  = 0.16427 
    # segment.percent_z_location                  = 0.01148 
    # segment.height                              = 3.92870 
    # segment.width                               = 3.71043 
    # fuselage.Segments.append(segment)    
    
    # # Segment                                   
    # segment                                     = SUAVE.Components.Fuselages.Segment()
    # segment.tag                                 = 'segment_8'   
    # segment.percent_x_location                  = 0.22478 
    # segment.percent_z_location                  = 0.01148 
    # segment.height                              = 3.92870 
    # segment.width                               = 3.92870 
    # fuselage.Segments.append(segment)   
    
    # # Segment                                   
    # segment                                     = SUAVE.Components.Fuselages.Segment()
    # segment.tag                                 = 'segment_9'     
    # segment.percent_x_location                  = 0.69164 
    # segment.percent_z_location                  = 0.01292
    # segment.height                              = 3.81957
    # segment.width                               = 3.81957
    # fuselage.Segments.append(segment)     
        
    # # Segment                                   
    # segment                                     = SUAVE.Components.Fuselages.Segment()
    # segment.tag                                 = 'segment_10'     
    # segment.percent_x_location                  = 0.71758 
    # segment.percent_z_location                  = 0.01292
    # segment.height                              = 3.81957
    # segment.width                               = 3.81957
    # fuselage.Segments.append(segment)   
        
    # # Segment                                   
    # segment                                     = SUAVE.Components.Fuselages.Segment()
    # segment.tag                                 = 'segment_11'     
    # segment.percent_x_location                  = 0.78098 
    # segment.percent_z_location                  = 0.01722
    # segment.height                              = 3.49217
    # segment.width                               = 3.71043
    # fuselage.Segments.append(segment)    
        
    # # Segment                                   
    # segment                                     = SUAVE.Components.Fuselages.Segment()
    # segment.tag                                 = 'segment_12'     
    # segment.percent_x_location                  = 0.85303
    # segment.percent_z_location                  = 0.02296
    # segment.height                              = 3.05565
    # segment.width                               = 3.16478
    # fuselage.Segments.append(segment)             
        
    # # Segment                                   
    # segment                                     = SUAVE.Components.Fuselages.Segment()
    # segment.tag                                 = 'segment_13'     
    # segment.percent_x_location                  = 0.91931 
    # segment.percent_z_location                  = 0.03157
    # segment.height                              = 2.40087
    # segment.width                               = 1.96435
    # fuselage.Segments.append(segment)               
        
    # # Segment                                   
    # segment                                     = SUAVE.Components.Fuselages.Segment()
    # segment.tag                                 = 'segment_14'     
    # segment.percent_x_location                  = 1.00 
    # segment.percent_z_location                  = 0.04593
    # segment.height                              = 1.09130
    # segment.width                               = 0.21826
    # fuselage.Segments.append(segment)           
    
    # add to vehicle
    vehicle.append_component(fuselage)


    #---------------------------------------------------------------------------------------------
    # DEFINE PROPELLER
    #---------------------------------------------------------------------------------------------
    # build network
    net = Battery_Propeller()
    net.number_of_propeller_engines  = 1 

    # Component 1 the ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc        = esc

    # Component 2 the Propeller 
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_of_blades       = 3.0
    prop.freestream_velocity    = 90.*Units.knots
    prop.angular_velocity       = 1900.  * Units.rpm
    prop.tip_radius             = 53. * Units.inches
    prop.hub_radius             = 0.15     * Units.inches
    prop.design_Cl              = 0.4
    prop.design_altitude        = 5000. * Units.feet
    prop.design_thrust          = None
    prop.design_power           = 600. * Units.horsepower
    # prop.origin                 = [[2.,2.5,0.784]]
    prop.variable_pitch         = True 
    prop.airfoil_geometry       =  ['../Airfoils/NACA_4412.txt']
    prop.airfoil_polars         = [['../Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                    '../Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                    '../Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                    '../Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                    '../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]

    prop.airfoil_polar_stations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    prop                        = propeller_design(prop)
    
    net.propellers.append(prop)


    # Component 3 the Battery
    # net                           = SUAVE.Components.Energy.Networks.Battery_Cell_Cycler()
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    net.tag                       ='battery_cell'
    bat.mass_properties.mass = 500. * Units.kg  
    bat.max_voltage          = 800. 
    initialize_from_mass(bat)
    net.battery              = bat
    net.voltage              = bat.max_voltage

    # Component 4 Miscellaneous Systems
    sys = SUAVE.Components.Systems.System()
    sys.mass_properties.mass = 5 # kg
 
    # Component 5 the Motor  
    motor                         = SUAVE.Components.Energy.Converters.Motor()
    motor.efficiency              = 0.95
    motor.gearbox_efficiency      = 1.
    # motor.origin                  = [[2.,  2.5, 0.784]]
    motor.nominal_voltage         = bat.max_voltage *3/4
    motor.propeller_radius        = prop.tip_radius
    motor.no_load_current         = 330.0
    motor                         = size_optimal_motor(motor,prop)
    motor.mass_properties.mass    = 200. * Units.kg 

    # Component 6 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 10. # Watts
    payload.mass_properties.mass = 1.0 * Units.kg
    net.payload                  = payload

    # Component 7 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 20. # Watts
    net.avionics        = avionics

    # add the solar network to the vehicle
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
    configs                                                    = SUAVE.Components.Configs.Config.Container() 
    base_config                                                = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag                                            = 'base'
    base_config.networks.battery_propeller.pitch_command = 0
    configs.append(base_config)
    
#     # ------------------------------------------------------------------
#     #   Cruise Configuration
#     # ------------------------------------------------------------------ 
#     config                                                     = SUAVE.Components.Configs.Config(base_config)
#     config.tag                                                 = 'cruise'
#     config.wings.main_wing.flaps.angle                         = 00 * Units['degree']
#     configs.append(config)
    
    
#     # ------------------------------------------------------------------
#     #   Takeoff Configuration
#     # ------------------------------------------------------------------ 
#     config                                                     = SUAVE.Components.Configs.Config(base_config)
#     config.tag                                                 = 'takeoff' 
#     config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
#     config.propulsors.network.propeller_speed                  = 1900 * Units['rpm']
# #    config.V2_VS_ratio                                         = 1.21
# #    config.maximum_lift_coefficient                            = 2.
    
#     configs.append(config)


#     # ------------------------------------------------------------------
#     #   Approach Configurations
#     # ------------------------------------------------------------------
    
#     config = SUAVE.Components.Configs.Config(base_config)
#     config.tag                                                 = 'approach'
#     config.wings.main_wing.flaps.angle                         = 30 * Units['degree']
#     config.propulsors.network.propeller_speed                  = 1750 * Units['rpm']
#     configs.append(config)
    
    
#     # ------------------------------------------------------------------
#     #   Landing Configuration
#     # ------------------------------------------------------------------

#     config                                                     = SUAVE.Components.Configs.Config(base_config)
#     config.tag                                                 = 'landing' 
#     config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
#     config.Vref_VS_ratio                                       = 1.23
#     config.maximum_lift_coefficient                            = 2.
                                                               
#     configs.append(config)


#     # ------------------------------------------------------------------
#     #   Stall Configuration
#     # ------------------------------------------------------------------

#     config = SUAVE.Components.Configs.Config(base_config)
#     config.tag                                                 = 'stall'
#     config.wings.main_wing.flaps.angle                         = 30 * Units['degree']
#     config.propulsors.network.propeller_speed                  = 1900 * Units['rpm']    
#     configs.append(config)
    
    
    return configs