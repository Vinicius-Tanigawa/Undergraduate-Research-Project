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

    # Mass properties
    vehicle.mass_properties.max_takeoff         = 8750. * Units.pounds
    vehicle.mass_properties.takeoff             = 8750. * Units.pounds
    vehicle.mass_properties.ramp                = 8785 * Units.pounds
    vehicle.mass_properties.cargo               = 0. 
    vehicle.mass_properties.max_landing         = 8500 * Units.pounds
#    vehicle.mass_properties.max_zero_fuel             = 62732.0
#    vehicle.mass_properties.operating_empty           = 62746.4   # kg                                                      
#    vehicle.design_cruise_alt                         = 35000.0 * Units.ft
#    vehicle.design_range                              = 3582 * Units.miles
#    vehicle.design_dynamic_pressure             = ( .5 *freestream0.density*(cruise_speed*cruise_speed))[0][0]
#    vehicle.design_mach_number                  =  mach_number    
#    vehicle.mass_properties.center_of_gravity         = [[ 15.30987849,   0.        ,  -0.48023939]]
#    vehicle.mass_properties.moments_of_inertia.tensor = [[3173074.17, 0 , 28752.77565],[0 , 3019041.443, 0],[0, 0, 5730017.433]]
    
    # Envelope properties
    vehicle.envelope.ultimate_load              = 3.8
#    vehicle.envelope.limit_load                 = 3.8  

    # Basic parameters
    vehicle.reference_area                      = 25.96     
    vehicle.passengers                          = 11.
    vehicle.systems.control                     = "fully powered"
    vehicle.systems.accessories                 = "medium range"


    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    wing                                        = SUAVE.Components.Wings.Main_Wing()
    wing.tag                                    = 'main_wing'    

    wing.taper                                  = 0.586
    wing.aspect_ratio                           = 15.875 ** 2 / 25.96
    wing.sweeps.quarter_chord                   = 2.93 * Units.deg
    wing.thickness_to_chord                     = 0.15

    wing.spans.projected                        = 15.875 

    wing.chords.root                            = 2 * wing.areas.reference / (wing.spans.projected * (1 + wing.taper))
    wing.chords.tip                             = wing.chords.root * wing.taper
    wing.chords.mean_aerodynamic                = (wing.chords.root + wing.chords.tip) / 2

    wing.areas.reference                        = vehicle.reference_area
#    wing.areas.wetted            = 225.08

    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees

#    wing.origin                                 = [[80.* Units.inches,0,0]]
#    wing.aerodynamic_center                     = [22.* Units.inches,0,0]

    wing.vertical                               = False
    wing.symmetric                              = True
    wing.high_lift                              = True 

#    wing.dynamic_pressure_ratio                 = 1.0 

    # Wing Segments
#    root_airfoil                          = SUAVE.Components.Wings.Airfoils.Airfoil()
#    root_airfoil.coordinate_file          = '../Vehicles/Airfoils/B737a.txt'
#    segment                               = SUAVE.Components.Wings.Segment()
#    segment.tag                           = 'Root'
#    segment.percent_span_location         = 0.0
#    segment.twist                         = 4. * Units.deg
#    segment.root_chord_percent            = 1.
#    segment.thickness_to_chord            = 0.1
#    segment.dihedral_outboard             = 2.5 * Units.degrees
#    segment.sweeps.quarter_chord          = 28.225 * Units.degrees
#    segment.thickness_to_chord            = .1
#    segment.append_airfoil(root_airfoil)
#    wing.append_segment(segment)
#
#    yehudi_airfoil                        = SUAVE.Components.Wings.Airfoils.Airfoil()
#    yehudi_airfoil.coordinate_file        = '../Vehicles/Airfoils/B737b.txt'
#    segment                               = SUAVE.Components.Wings.Segment()
#    segment.tag                           = 'Yehudi'
#    segment.percent_span_location         = 0.324
#    segment.twist                         = 0.047193 * Units.deg
#    segment.root_chord_percent            = 0.5
#    segment.thickness_to_chord            = 0.1
#    segment.dihedral_outboard             = 5.5 * Units.degrees
#    segment.sweeps.quarter_chord          = 25. * Units.degrees
#    segment.thickness_to_chord            = .1
#    segment.append_airfoil(yehudi_airfoil)
#    wing.append_segment(segment)
#
#    mid_airfoil                           = SUAVE.Components.Wings.Airfoils.Airfoil()
#    mid_airfoil.coordinate_file           = '../Vehicles/Airfoils/B737c.txt'
#    segment                               = SUAVE.Components.Wings.Segment()
#    segment.tag                           = 'Section_2'
#    segment.percent_span_location         = 0.963
#    segment.twist                         = 0.00258 * Units.deg
#    segment.root_chord_percent            = 0.220
#    segment.thickness_to_chord            = 0.1
#    segment.dihedral_outboard             = 5.5 * Units.degrees
#    segment.sweeps.quarter_chord          = 56.75 * Units.degrees
#    segment.thickness_to_chord            = .1
#    segment.append_airfoil(mid_airfoil)
#    wing.append_segment(segment)
#
#    tip_airfoil                           =  SUAVE.Components.Wings.Airfoils.Airfoil()
#    tip_airfoil.coordinate_file           = '../Vehicles/Airfoils/B737d.txt'
#    segment                               = SUAVE.Components.Wings.Segment()
#    segment.tag                           = 'Tip'
#    segment.percent_span_location         = 1.
#    segment.twist                         = 0. * Units.degrees
#    segment.root_chord_percent            = 0.10077
#    segment.thickness_to_chord            = 0.1
#    segment.dihedral_outboard             = 0.
#    segment.sweeps.quarter_chord          = 0.
#    segment.thickness_to_chord            = .1
#    segment.append_airfoil(tip_airfoil)
#    wing.append_segment(segment)
                                         
    # control surfaces -------------------------------------------
    flap                                        = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                                    = 'flap' 
    flap.span_fraction_start                    = 1.71 / 2
    flap.span_fraction_end                      = flap.span_fraction_start + 4.84  
    flap.deflection                             = 0.0 * Units.degrees
    flap.chord_fraction                         = 0.41 
    flap.configuration_type                     = 'single_slotted'    
    wing.append_control_surface(flap)           

#    aileron                       = SUAVE.Components.Wings.Control_Surfaces.Aileron()
#    aileron.tag                   = 'aileron'
#    aileron.span_fraction_start   = 0.7
#    aileron.span_fraction_end     = 0.963
#    aileron.deflection            = 0.0 * Units.degrees
#    aileron.chord_fraction        = 0.16
#    wing.append_control_surface(aileron)

    # Add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        
                                                
    wing                                        = SUAVE.Components.Wings.Wing()
    wing.tag                                    = 'horizontal_stabilizer' 

    wing.aspect_ratio                           = 2.95
    wing.sweeps.quarter_chord                   = 2.05 * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.taper                                  = 0.62

    wing.spans.projected                        = 6.25

    wing.chords.root                            = 1.31
    wing.chords.tip                             = 0.81
    wing.chords.mean_aerodynamic                = (wing.chords.root + wing.chords.tip)/2 

    wing.areas.reference                        = 13.22
#    wing.areas.exposed           = 59.354    # Exposed area of the horizontal tail
#    wing.areas.wetted            = 71.81     # Wetted area of the horizontal tail

    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees

#    wing.origin                                 = [[246.* Units.inches,0,0]]
#    wing.aerodynamic_center                     = [20.* Units.inches,0,0]

    wing.vertical                               = False
    wing.symmetric                              = True

#    wing.dynamic_pressure_ratio                 = 0.9

    # Wing Segments
#    segment                        = SUAVE.Components.Wings.Segment()
#    segment.tag                    = 'root_segment'
#    segment.percent_span_location  = 0.0
#    segment.twist                  = 0. * Units.deg
#    segment.root_chord_percent     = 1.0
#    segment.dihedral_outboard      = 8.63 * Units.degrees
#    segment.sweeps.quarter_chord   = 28.2250  * Units.degrees 
#    segment.thickness_to_chord     = .1
#    wing.append_segment(segment)

#    segment                        = SUAVE.Components.Wings.Segment()
#    segment.tag                    = 'tip_segment'
#    segment.percent_span_location  = 1.
#    segment.twist                  = 0. * Units.deg
#    segment.root_chord_percent     = 0.3333               
#    segment.dihedral_outboard      = 0 * Units.degrees
#    segment.sweeps.quarter_chord   = 0 * Units.degrees  
#    segment.thickness_to_chord     = .1
#    wing.append_segment(segment)

    # control surfaces -------------------------------------------
#    elevator                       = SUAVE.Components.Wings.Control_Surfaces.Elevator()
#    elevator.tag                   = 'elevator'
#    elevator.span_fraction_start   = 0.09
#    elevator.span_fraction_end     = 0.92
#    elevator.deflection            = 0.0  * Units.deg
#    elevator.chord_fraction        = 0.3
#    wing.append_control_surface(elevator)

    # Add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing                                        = SUAVE.Components.Wings.Wing()
    wing.tag                                    = 'vertical_stabilizer' 

    wing.aspect_ratio                           = 2.0
    wing.sweeps.quarter_chord                   = 14.06 * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.taper                                  = 0.38

#    wing.total_lenght                           = 4.21
    wing.spans.projected                        = 3.05

    wing.chords.root                            = 2.21
    wing.chords.tip                             = 0.84
    wing.chords.mean_aerodynamic                = (wing.chords.root + wing.chords.tip) / 2 

    wing.areas.reference                        = 4.21
#    wing.areas.wetted                           = 57.25

    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees

#    wing.origin                                 = [[237.* Units.inches,0,0]]
#    wing.aerodynamic_center                     = [20.* Units.inches,0,0] 

    wing.vertical                               = True 
    wing.symmetric                              = False
#    wing.t_tail                                 = False 

#    wing.dynamic_pressure_ratio                 = 1.0

    # Wing Segments
#    segment                               = SUAVE.Components.Wings.Segment()
#    segment.tag                           = 'root'
#    segment.percent_span_location         = 0.0
#    segment.twist                         = 0. * Units.deg
#    segment.root_chord_percent            = 1.
#    segment.dihedral_outboard             = 0 * Units.degrees
#    segment.sweeps.quarter_chord          = 61.485 * Units.degrees  
#    segment.thickness_to_chord            = .1
#    wing.append_segment(segment)

#    segment                               = SUAVE.Components.Wings.Segment()
#    segment.tag                           = 'segment_1'
#    segment.percent_span_location         = 0.2962
#    segment.twist                         = 0. * Units.deg
#    segment.root_chord_percent            = 0.45
#    segment.dihedral_outboard             = 0. * Units.degrees
#    segment.sweeps.quarter_chord          = 31.2 * Units.degrees   
#    segment.thickness_to_chord            = .1
#    wing.append_segment(segment)

#    segment                               = SUAVE.Components.Wings.Segment()
#    segment.tag                           = 'segment_2'
#    segment.percent_span_location         = 1.0
#    segment.twist                         = 0. * Units.deg
#    segment.root_chord_percent            = 0.1183 
#    segment.dihedral_outboard             = 0.0 * Units.degrees
#    segment.sweeps.quarter_chord          = 0.0    
#    segment.thickness_to_chord            = .1  
#    wing.append_segment(segment)

    # Add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage'

#    fuselage.number_coach_seats                 = 4.    
#    fuselage.seats_abreast                      = 2.
#    fuselage.seat_pitch            = 31. * Units.inches
#    fuselage.fineness.nose                      = 1.6
#    fuselage.fineness.tail                      = 2.    

#    fuselage.lengths.nose                       = 60.  * Units.inches
#    fuselage.lengths.tail          = 8.0  
    fuselage.lengths.total                      = 16.67
#    fuselage.lengths.cabin                      = 105. * Units.inches
#    fuselage.lengths.fore_space    = 6.
#    fuselage.lengths.aft_space     = 5.
#
    fuselage.width                              = 1.71

    fuselage.heights.maximum                    = 2.10 + 0.68
    fuselage.heights.at_quarter_length          = 2.10 + 0.68 #Testing, not real value
    fuselage.heights.at_three_quarters_length   = 2.10 + 0.68 #Testing, not real value
    fuselage.heights.at_wing_root_quarter_chord = 0.68 #Testing, not real value

    fuselage.areas.front_projected              = fuselage.width* fuselage.heights.maximum
    fuselage.areas.side_projected               = fuselage.lengths.total* fuselage.heights.maximum
    fuselage.areas.wetted                       = 75.58

#    fuselage.effective_diameter    = 3.74 

#    fuselage.differential_pressure              = 8*Units.psi

    # Segment  
#    segment                                     = SUAVE.Components.Fuselages.Segment() 
#    segment.tag                                 = 'segment_0'    
#    segment.percent_x_location                  = 0.0000
#    segment.percent_z_location                  = -0.00144 
#    segment.height                              = 0.0100 
#    segment.width                               = 0.0100  
#    fuselage.Segments.append(segment)   
#    
#    # Segment  
#    segment                                     = SUAVE.Components.Fuselages.Segment() 
#    segment.tag                                 = 'segment_1'    
#    segment.percent_x_location                  = 0.00576 
#    segment.percent_z_location                  = -0.00144 
#    segment.height                              = 0.7500
#    segment.width                               = 0.6500
#    fuselage.Segments.append(segment)   
#    
#    # Segment                                   
#    segment                                     = SUAVE.Components.Fuselages.Segment()
#    segment.tag                                 = 'segment_2'   
#    segment.percent_x_location                  = 0.02017 
#    segment.percent_z_location                  = 0.00000 
#    segment.height                              = 1.52783 
#    segment.width                               = 1.20043 
#    fuselage.Segments.append(segment)      
#    
#    # Segment                                   
#    segment                                     = SUAVE.Components.Fuselages.Segment()
#    segment.tag                                 = 'segment_3'   
#    segment.percent_x_location                  = 0.03170 
#    segment.percent_z_location                  = 0.00000 
#    segment.height                              = 1.96435 
#    segment.width                               = 1.52783 
#    fuselage.Segments.append(segment)   
#
#    # Segment                                   
#    segment                                     = SUAVE.Components.Fuselages.Segment()
#    segment.tag                                 = 'segment_4'   
#    segment.percent_x_location                  = 0.04899 	
#    segment.percent_z_location                  = 0.00431 
#    segment.height                              = 2.72826 
#    segment.width                               = 1.96435 
#    fuselage.Segments.append(segment)   
#    
#    # Segment                                   
#    segment                                     = SUAVE.Components.Fuselages.Segment()
#    segment.tag                                 = 'segment_5'   
#    segment.percent_x_location                  = 0.07781 
#    segment.percent_z_location                  = 0.00861 
#    segment.height                              = 3.49217 
#    segment.width                               = 2.61913 
#    fuselage.Segments.append(segment)     
#    
#    # Segment                                   
#    segment                                     = SUAVE.Components.Fuselages.Segment()
#    segment.tag                                 = 'segment_6'   
#    segment.percent_x_location                  = 0.10375 
#    segment.percent_z_location                  = 0.01005 
#    segment.height                              = 3.70130 
#    segment.width                               = 3.05565 
#    fuselage.Segments.append(segment)             
#     
#    # Segment                                   
#    segment                                     = SUAVE.Components.Fuselages.Segment()
#    segment.tag                                 = 'segment_7'   
#    segment.percent_x_location                  = 0.16427 
#    segment.percent_z_location                  = 0.01148 
#    segment.height                              = 3.92870 
#    segment.width                               = 3.71043 
#    fuselage.Segments.append(segment)    
#    
#    # Segment                                   
#    segment                                     = SUAVE.Components.Fuselages.Segment()
#    segment.tag                                 = 'segment_8'   
#    segment.percent_x_location                  = 0.22478 
#    segment.percent_z_location                  = 0.01148 
#    segment.height                              = 3.92870 
#    segment.width                               = 3.92870 
#    fuselage.Segments.append(segment)   
#    
#    # Segment                                   
#    segment                                     = SUAVE.Components.Fuselages.Segment()
#    segment.tag                                 = 'segment_9'     
#    segment.percent_x_location                  = 0.69164 
#    segment.percent_z_location                  = 0.01292
#    segment.height                              = 3.81957
#    segment.width                               = 3.81957
#    fuselage.Segments.append(segment)     
#        
#    # Segment                                   
#    segment                                     = SUAVE.Components.Fuselages.Segment()
#    segment.tag                                 = 'segment_10'     
#    segment.percent_x_location                  = 0.71758 
#    segment.percent_z_location                  = 0.01292
#    segment.height                              = 3.81957
#    segment.width                               = 3.81957
#    fuselage.Segments.append(segment)   
#        
#    # Segment                                   
#    segment                                     = SUAVE.Components.Fuselages.Segment()
#    segment.tag                                 = 'segment_11'     
#    segment.percent_x_location                  = 0.78098 
#    segment.percent_z_location                  = 0.01722
#    segment.height                              = 3.49217
#    segment.width                               = 3.71043
#    fuselage.Segments.append(segment)    
#        
#    # Segment                                   
#    segment                                     = SUAVE.Components.Fuselages.Segment()
#    segment.tag                                 = 'segment_12'     
#    segment.percent_x_location                  = 0.85303
#    segment.percent_z_location                  = 0.02296
#    segment.height                              = 3.05565
#    segment.width                               = 3.16478
#    fuselage.Segments.append(segment)             
#        
#    # Segment                                   
#    segment                                     = SUAVE.Components.Fuselages.Segment()
#    segment.tag                                 = 'segment_13'     
#    segment.percent_x_location                  = 0.91931 
#    segment.percent_z_location                  = 0.03157
#    segment.height                              = 2.40087
#    segment.width                               = 1.96435
#    fuselage.Segments.append(segment)               
#        
#    # Segment                                   
#    segment                                     = SUAVE.Components.Fuselages.Segment()
#    segment.tag                                 = 'segment_14'     
#    segment.percent_x_location                  = 1.00 
#    segment.percent_z_location                  = 0.04593
#    segment.height                              = 1.09130
#    segment.width                               = 0.21826
#    fuselage.Segments.append(segment)                  
    
    # add to vehicle
    vehicle.append_component(fuselage)
    
    # ------------------------------------------------------------------
    #   Landing gear
    # ------------------------------------------------------------------  

    landing_gear                                = SUAVE.Components.Landing_Gear.Landing_Gear()

    landing_gear.tag = 'main_landing_gear'
    landing_gear.main_tire_diameter   = 0.6
#    landing_gear.main_strut_length        = 1.8 * Units.m
#    landing_gear.main_units               = 1
#    landing_gear.main_wheels              = 2

    landing_gear.nose_tire_diameter   = 0.5
#    landing_gear.nose_strut_length        = 1.8 * Units.m
#    landing_gear.nose_units               = 1
#    landing_gear.nose_wheels              = 2
                                                                           
    vehicle.landing_gear                        = landing_gear
                                                
    # ------------------------------------------------------------------
    #   Fuel
    # ------------------------------------------------------------------    

#    fuel                                        = SUAVE.Attributes.Physical_Component()
#    fuel.origin                                 = wing.origin
#    fuel.mass_properties.center_of_gravity      = wing.mass_properties.center_of_gravity
#    fuel.mass_properties.mass                   = 319 *Units.lbs
#    vehicle.fuel                                = fuel

    # ------------------------------------------------------------------
    #   Propulsor
    # ------------------------------------------------------------------    
    
    # build network
    net                                         = SUAVE.Components.Energy.Networks.Turboprop()
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
    net.engine.torque_limit                     = 1970 * Units['lbf*ft']
#    net.engine.rated_sfc                        = 0.64 * Units['lb/hp/h'] * 1.02
    net.engine.power_specific_fuel_consumption  = 0.64
    
    # the prop
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_of_blades        = 3.0
    prop.freestream_velocity     = 200.   * Units.knots
    prop.angular_velocity        = 1900.  * Units.rpm
    prop.tip_radius              = 53. * Units.inches
    prop.hub_radius              = 0.15     * Units.meters
    prop.design_Cl               = 0.4
    prop.design_altitude         = 20000. * Units.feet
    prop.design_thrust           = None
    prop.design_power            = 600. * Units.horsepower
    prop.activity_factor         = 90
    prop.weight_class            = 0

    prop.airfoil_geometry        =  ['./Airfoils/NACA_4412.txt'] 
    prop.airfoil_polars          = [['./Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                     './Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                     './Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                     './Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                     './Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]

    prop.airfoil_polar_stations  = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]   
   
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