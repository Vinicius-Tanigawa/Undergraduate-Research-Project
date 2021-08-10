# Embraer_190
#
# Created:  Feb 2017, M. Vegh (data taken from Embraer_E190_constThr/mission_Embraer_E190_constThr, and Regional_Jet_Optimization/Vehicles2.py), takeoff_field_length/takeoff_field_length.py, landing_field_length/landing_field_length.py
# Modified: Mar 2020, M. Clarke
#           May 2020, E. Botero


""" setup file for the E190 vehicle
"""


# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import numpy as np
import SUAVE

from SUAVE.Core import Units
from SUAVE.Core import Data 
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_planform
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion import propeller_design 
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_energy_and_power, initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv    
from SUAVE.Plots.Geometry_Plots.plot_vehicle import plot_vehicle
# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Embraer_E190AR'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    # mass properties (http://www.embraercommercialaviation.com/AircraftPDF/E190_Weights.pdf)
    vehicle.mass_properties.max_takeoff               = 51800.   # kg
    vehicle.mass_properties.operating_empty           = 27837.   # kg
    vehicle.mass_properties.takeoff                   = 51800.   # kg
    vehicle.mass_properties.max_zero_fuel             = 40900.   # kg
    vehicle.mass_properties.max_payload               = 13063.   # kg
    vehicle.mass_properties.max_fuel                  = 12971.   # kg
    vehicle.mass_properties.cargo                     =     0.0  # kg

    vehicle.mass_properties.center_of_gravity         = [[16.8, 0, 1.6]]
    vehicle.mass_properties.moments_of_inertia.tensor = [[10 ** 5, 0, 0],[0, 10 ** 6, 0,],[0,0, 10 ** 7]] 

    # envelope properties
    vehicle.envelope.ultimate_load = 3.5
    vehicle.envelope.limit_load    = 1.5

    # basic parameters
    vehicle.reference_area         = 92.
    vehicle.passengers             = 106
    vehicle.systems.control        = "fully powered"
    vehicle.systems.accessories    = "medium range"


    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
    wing                         = SUAVE.Components.Wings.Main_Wing()
    wing.tag                     = 'main_wing'
    wing.areas.reference         = 92.0
    wing.aspect_ratio            = 8.4
    wing.chords.root             = 6.2
    wing.chords.tip              = 1.44
    wing.sweeps.quarter_chord    = 23.0 * Units.deg
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.28
    wing.dihedral                = 5.00 * Units.deg
    wing.spans.projected         = 28.72
    wing.origin                  = [[13.0,0,-1.50]]
    wing.vertical                = False
    wing.symmetric               = True       
    wing.high_lift               = True
    wing.areas.exposed           = 0.80 * wing.areas.wetted        
    wing.twists.root             = 2.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 1.0
    
    
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'root'
    segment.percent_span_location = 0.0
    segment.twist                 = 4. * Units.deg
    segment.root_chord_percent    = 1.
    segment.thickness_to_chord    = .11
    segment.dihedral_outboard     = 5. * Units.degrees
    segment.sweeps.quarter_chord  = 20.6 * Units.degrees
    wing.Segments.append(segment)    
    
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'yehudi'
    segment.percent_span_location = 0.348
    segment.twist                 = (4. - segment.percent_span_location*4.) * Units.deg
    segment.root_chord_percent    = 0.60
    segment.thickness_to_chord    = .11
    segment.dihedral_outboard     = 4 * Units.degrees
    segment.sweeps.quarter_chord  = 24.1 * Units.degrees
    wing.Segments.append(segment)
    
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'section_2'
    segment.percent_span_location = 0.961
    segment.twist                 = (4. - segment.percent_span_location*4.) * Units.deg
    segment.root_chord_percent    = 0.25
    segment.thickness_to_chord    = .11
    segment.dihedral_outboard     = 70. * Units.degrees
    segment.sweeps.quarter_chord  = 50. * Units.degrees
    wing.Segments.append(segment)

    segment = SUAVE.Components.Wings.Segment() 
    segment.tag                   = 'Tip'
    segment.percent_span_location = 1.
    segment.twist                 = (4. - segment.percent_span_location*4.) * Units.deg
    segment.root_chord_percent    = 0.070
    segment.thickness_to_chord    = .11
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 0.
    wing.Segments.append(segment)            

    # control surfaces -------------------------------------------
    flap                       = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                   = 'flap' 
    flap.span_fraction_start   = 0.11
    flap.span_fraction_end     = 0.85
    flap.deflection            = 0.0 * Units.deg 
    flap.chord_fraction        = 0.28    
    flap.configuration_type    = 'double_slotted'
    wing.append_control_surface(flap)   
        
    slat                       = SUAVE.Components.Wings.Control_Surfaces.Slat()
    slat.tag                   = 'slat' 
    slat.span_fraction_start   = 0.324 
    slat.span_fraction_end     = 0.963     
    slat.deflection            = 1.0 * Units.deg 
    slat.chord_fraction        = 0.1   
    wing.append_control_surface(slat) 
    
    wing                         = wing_planform(wing)
    
    wing.areas.exposed           = 0.80 * wing.areas.wetted
    wing.twists.root             = 2.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 1.0   

    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'
    wing.areas.reference         = 26.0
    wing.aspect_ratio            = 5.5
    wing.sweeps.quarter_chord    = 34.5 * Units.deg
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.11
    wing.dihedral                = 8.4 * Units.degrees
    wing.origin                  = [[31,0,0.44]]
    wing.vertical                = False
    wing.symmetric               = True       
    wing.high_lift               = False  
    wing                         = wing_planform(wing)
    wing.areas.exposed           = 0.9 * wing.areas.wetted 
    wing.twists.root             = 2.0 * Units.degrees
    wing.twists.tip              = 2.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 0.90

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'
    wing.areas.reference         = 16.0
    wing.aspect_ratio            =  1.7
    wing.sweeps.quarter_chord    = 35. * Units.deg
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.31
    wing.dihedral                = 0.00
    wing.origin                  = [[30.4,0,1.675]]
    wing.vertical                = True
    wing.symmetric               = False       
    wing.high_lift               = False
    wing                         = wing_planform(wing)
    wing.areas.exposed           = 0.9 * wing.areas.wetted
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 1.00
    
    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag    = 'fuselage'
    fuselage.origin = [[0,0,0]]
    fuselage.number_coach_seats    = vehicle.passengers
    fuselage.seats_abreast         = 4
    fuselage.seat_pitch            = 30. * Units.inches

    fuselage.fineness.nose         = 1.28
    fuselage.fineness.tail         = 3.48

    fuselage.lengths.nose          = 6.0
    fuselage.lengths.tail          = 9.0
    fuselage.lengths.cabin         = 21.24
    fuselage.lengths.total         = 36.24
    fuselage.lengths.fore_space    = 0.
    fuselage.lengths.aft_space     = 0.

    fuselage.width                 = 3.01 * Units.meters

    fuselage.heights.maximum       = 3.35    
    fuselage.heights.at_quarter_length          = 3.35 
    fuselage.heights.at_three_quarters_length   = 3.35 
    fuselage.heights.at_wing_root_quarter_chord = 3.35 

    fuselage.areas.side_projected  = 239.20
    fuselage.areas.wetted          = 327.01
    fuselage.areas.front_projected = 8.0110

    fuselage.effective_diameter    = 3.18

    fuselage.differential_pressure = 10**5 * Units.pascal    # Maximum differential pressure

    # add to vehicle
    vehicle.append_component(fuselage)

    #---------------------------------------------------------------------------------------------
    # DEFINE PROPELLER
    #---------------------------------------------------------------------------------------------
    # build network    
    net = Battery_Propeller() 
    net.number_of_engines       = 2.
    net.nacelle_diameter        = 0.95  * Units.meter #42 * Units.inches
    net.engine_length           = 2.25  * Units.meter #0.01 * Units.inches
    net.areas                   = Data()
    net.areas.wetted            = 2.0*np.pi*net.nacelle_diameter*net.engine_length  


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
    bat.mass_properties.mass = 5000. * Units.kg  
    bat.specific_energy      = 207. * Units.Wh/Units.kg
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
    motor.gearbox_efficiency           = 1.    
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
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.V2_VS_ratio = 1.21
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.Vref_VS_ratio = 1.23
    configs.append(config)   
     
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'    
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.V2_VS_ratio = 1.21
    
    # payload?
    
    configs.append(config)

    # done!
    return configs
