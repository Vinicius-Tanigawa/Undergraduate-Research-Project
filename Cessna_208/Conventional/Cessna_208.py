# Cessna_208.py


# Created:  Jun 2021, V. Tanigawa 


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

    # Vehicle level Mass Properties -------------------------------------------
    vehicle.mass_properties.max_takeoff               = 3629. * Units.kilogram 
    # vehicle.mass_properties.takeoff                   = 3645. * Units.kilogram  
    vehicle.mass_properties.takeoff                   = 8600. * Units.pound # Calibration
    vehicle.mass_properties.operating_empty           = 1832. * Units.kilogram 
    vehicle.mass_properties.max_zero_fuel             = 2351.0 * Units.kilogram
    vehicle.mass_properties.cargo                     = 1400.  * Units.kilogram   
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

    wing.aspect_ratio            = 9.71 # (wing span² / wing area)
    wing.sweeps.quarter_chord    = 2.74 * Units.deg # (cotan((root chord - tip chord) / wing span))
    wing.thickness_to_chord      = 0.195 #(wing thickness / chord ratio)
    wing.taper                   = 0.616 # (tip chord / root chord)

    wing.spans.projected         = 15.875 * Units.meter 

    wing.chords.root             = 1.98 * Units.meter 
    wing.chords.tip              = 1.22 * Units.meter 
    wing.chords.mean_aerodynamic = 1.63 * Units.meter # (root chord * 2/3 * ((1 + taper ratio + (taper ratio)²) / (1 + taper ratio))

    wing.areas.reference         = 25.96 * Units['meters**2'] # (((root chord + tip chord) * wing span) / 2)
    wing.areas.wetted            = 51.82 * Units['meters**2'] # (2 * area reference)

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[4.938,0,0.851]]
    wing.aerodynamic_center      = [0,0,0]  

    wing.vertical                = False 
    wing.symmetric               = True 
    wing.high_lift               = True

    wing.dynamic_pressure_ratio  = 1.0 

    #   Main Wing Segments -------------------------------------------
    #   Airfoil Geometry File (reference): https://m-selig.ae.illinois.edu/ads/aircraft.html
    #   Airfoil Data Acquired from: http://airfoiltools.com/airfoil/naca5digit?MNaca5DigitForm%5Bcl%5D=0.3&MNaca5DigitForm%5BposKey%5D=15_0&MNaca5DigitForm%5Bthick%5D=15.5&MNaca5DigitForm%5BnumPoints%5D=81&MNaca5DigitForm%5BcosSpace%5D=0&MNaca5DigitForm%5BcosSpace%5D=1&MNaca5DigitForm%5BcloseTe%5D=0&yt0=Plot
    root_airfoil                          = SUAVE.Components.Wings.Airfoils.Airfoil()
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

    mid_airfoil                           = SUAVE.Components.Wings.Airfoils.Airfoil()
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

    tip_airfoil                           =  SUAVE.Components.Wings.Airfoils.Airfoil()
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
    
    wing.aspect_ratio            = 2.953 # (wing span² / wing area)
    wing.sweeps.quarter_chord    = 4.575 * Units.deg # (cotan((root chord - tip chord) / wing span))
    wing.thickness_to_chord      = 0.12 #(wing thickness / chord ratio)
    wing.taper                   = 0.618 # (tip chord / root chord)

    wing.spans.projected         = 6.248 * Units.meter

    wing.chords.root             = 1.31  * Units.meter
    wing.chords.tip              = 0.81 * Units.meter
    wing.chords.mean_aerodynamic = 1.079  * Units.meter # (root chord * 2/3 * ((1 + taper ratio + (taper ratio)²) / (1 + taper ratio))

    wing.areas.reference         = 13.22   * Units['meters**2']
    wing.areas.wetted            = 26.44 * Units['meters**2']
    wing.areas.exposed           = 11.152 * Units['meters**2']
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  

    # wing.origin                  = [[33.02,0,1.466]]
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

    wing.aspect_ratio            = 1.21 # (wing span² / wing area)
    wing.sweeps.quarter_chord    = 24.189 * Units.deg # (cotan((root chord - tip chord) / wing span))
    wing.thickness_to_chord      = 0.12 #(wing thickness / chord ratio)
    wing.taper                   = 0.38 # (tip chord / root chord)

    wing.spans.projected         = 3.05 * Units.meter
    wing.total_length            = wing.spans.projected 

    wing.chords.root             = 2.21  * Units.meter
    wing.chords.tip              = 0.84  * Units.meter
    wing.chords.mean_aerodynamic = 1.627   * Units.meter # (root chord * 2/3 * ((1 + taper ratio + (taper ratio)²) / (1 + taper ratio))
    
    wing.areas.reference         = 4.21 * Units['meters**2']
    wing.areas.wetted            = 8.42 * Units['meters**2']
    
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees
    
    # wing.origin                  = [[26.944,0,1.54]]
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
    fuselage.fineness.nose         = 1.500 # Without cabin
    # fuselage.fineness.nose         = 2.133 # With cabin
    fuselage.fineness.tail         = 3.104

    fuselage.lengths.nose          = 2.285   * Units.meter # Without cabin
    # fuselage.lengths.nose          = 3.251   * Units.meter # With cabin
    fuselage.lengths.tail          = 4.529   * Units.meter
    # fuselage.lengths.cabin         = 28.85
    fuselage.lengths.total         = 16.67 * Units.meter
    fuselage.lengths.fore_space    = 0.    * Units.meter
    fuselage.lengths.aft_space     = 0.    * Units.meter

    fuselage.width                 = 1.878  * Units.meter

    fuselage.heights.maximum       = 1.684  * Units.meter
    fuselage.heights.at_quarter_length          = 1.684 * Units.meter
    fuselage.heights.at_three_quarters_length   = 1.643 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 1.684 * Units.meter

    fuselage.areas.side_projected  = 21.353 * Units['meters**2']
    fuselage.areas.wetted          = 75.58  * Units['meters**2']
    fuselage.areas.front_projected = 3.163    * Units['meters**2']

    fuselage.effective_diameter    = 1.781     * Units.meter

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
                                                
    # ------------------------------------------------------------------
    #   Fuel
    # ------------------------------------------------------------------    
    fuel                                        = SUAVE.Attributes.Propellants.Aviation_Gasoline()
    fuel.mass_properties                        = SUAVE.Components.Mass_Properties() 
    fuel.number_of_tanks                        = 2.
    fuel.origin                                 = wing.origin
    fuel.internal_volume                        = fuel.mass_properties.mass/fuel.density #all of the fuel volume is internal
    # fuel.mass_properties.center_of_gravity      = wing.mass_properties.center_of_gravity
    fuel.mass_properties.mass                   = 1009 * Units.kg
    vehicle.fuel                                = fuel

    # ------------------------------------------------------------------
    #   Propulsor
    # ------------------------------------------------------------------    


    # build network
    net                                         = SUAVE.Components.Energy.Networks.Internal_Combustion_Propeller()
    net.tag                                     = 'internal_combustion'
    net.number_of_engines                       = 1.
    net.nacelle_diameter                        = 0.2 * Units.meters
    net.engine_length                           = 0.01 * Units.meters
    net.areas                                   = Data()
    net.areas.wetted                            = 0.01
                                                
    # the engine                    
    net.engine                                  = SUAVE.Components.Energy.Converters.Internal_Combustion_Engine()
    net.engine.sea_level_power                  = 675. * Units.horsepower
    net.engine.flat_rate_altitude               = 0.0
    net.engine.rated_speed                      = 1900. * Units.rpm
    net.engine.power_specific_fuel_consumption  = 0.64
    
    
    # the prop
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_of_blades        = 2.0
    prop.freestream_velocity     = 200.   * Units.knots
    prop.angular_velocity        = 1850.  * Units.rpm
    prop.tip_radius              = 53. * Units.inches
    prop.hub_radius              = 0.15     * Units.inches
    prop.design_Cl               = 0.4
    prop.design_altitude         = 20000. * Units.feet
    prop.design_power            = 675. * Units.horsepower

    prop.airfoil_geometry        =  ['../Airfoils/NACA_4412.txt'] 
    prop.airfoil_polars          = [['../Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                     '../Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                     '../Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                     '../Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                     '../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]

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