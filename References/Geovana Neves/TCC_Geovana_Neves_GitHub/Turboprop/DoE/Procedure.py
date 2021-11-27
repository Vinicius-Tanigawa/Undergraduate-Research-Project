# Procedure.py
# 
# Created:  Mar 2016, M. Vegh
# Modified: Aug 2017, E. Botero

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import numpy as np
from copy import deepcopy

import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Analyses.Process import Process
from SUAVE.Methods.Propulsion.turbofan_sizing import turbofan_sizing
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Propulsion.compute_turbofan_geometry import compute_turbofan_geometry
from SUAVE.Methods.Aerodynamics.Fidelity_Zero.Lift.compute_max_lift_coeff import compute_max_lift_coeff
from SUAVE.Optimization.write_optimization_outputs import write_optimization_outputs
from SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform import wing_planform
from scipy import interpolate
# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------   

def setup():
    
    # ------------------------------------------------------------------
    #   Analysis Procedure
    # ------------------------------------------------------------------ 
    
    # size the base config
    procedure = Process()
    procedure.simple_sizing = simple_sizing
    
    # find the weights
    procedure.weights = weight
    # finalizes the data dependencies
    procedure.finalize = finalize

    # performance studies
    procedure.missions                   = Process()
    procedure.missions.design_mission    = design_mission

    # calculate field lengths
    procedure.takeoff_field_length = takeoff_field_length
    procedure.landing_field_length = landing_field_length

    # post process the results
    procedure.post_process = post_process
        
    return procedure

# ----------------------------------------------------------------------        
#   Design Mission
# ----------------------------------------------------------------------    
def design_mission(nexus):
    analyses = nexus.analyses
    mission = nexus.missions.base
    mission.design_range = 500. * Units.nautical_miles

    cruise_altitude = check_cruise_altitude(analyses)
    mission.segments['climb_10'].altitude_end = cruise_altitude

    # find_target_range(nexus, mission)

    results = nexus.results
    results.base = mission.evaluate()
    nexus.summary.cruise_altitude = cruise_altitude

    return nexus


def find_target_range(nexus, mission):
    segments = mission.segments
    cruise_altitude = mission.segments['climb_10'].altitude_end

    climb_1 = segments['climb_1']
    climb_2 = segments['climb_2']
    climb_3 = segments['climb_3']
    climb_4 = segments['climb_4']
    climb_5 = segments['climb_5']
    climb_6 = segments['climb_6']
    climb_7 = segments['climb_7']
    climb_8 = segments['climb_8']
    climb_9 = segments['climb_9']
    climb_10 = segments['climb_10']

    descent_1 = segments['descent_1']
    descent_2 = segments['descent_2']
    descent_3 = segments['descent_3']

    x_climb_1 = climb_1.altitude_end / np.tan(np.arcsin(climb_1.climb_rate / climb_1.air_speed))
    x_climb_2 = (climb_2.altitude_end - climb_1.altitude_end) / np.tan(
        np.arcsin(climb_2.climb_rate / climb_2.air_speed))
    x_climb_3 = (climb_3.altitude_end - climb_2.altitude_end) / np.tan(
        np.arcsin(climb_3.climb_rate / climb_3.air_speed))
    x_climb_4 = (climb_4.altitude_end - climb_3.altitude_end) / np.tan(
        np.arcsin(climb_4.climb_rate / climb_4.air_speed))
    x_climb_5 = (climb_5.altitude_end - climb_4.altitude_end) / np.tan(
        np.arcsin(climb_5.climb_rate / climb_5.air_speed))
    x_climb_6 = (climb_6.altitude_end - climb_5.altitude_end) / np.tan(
        np.arcsin(climb_6.climb_rate / climb_5.air_speed))
    x_climb_7 = (climb_7.altitude_end - climb_6.altitude_end) / np.tan(
        np.arcsin(climb_7.climb_rate / climb_6.air_speed))
    x_climb_8 = (climb_8.altitude_end - climb_7.altitude_end) / np.tan(
        np.arcsin(climb_8.climb_rate / climb_7.air_speed))
    x_climb_9 = (climb_9.altitude_end - climb_8.altitude_end) / np.tan(
        np.arcsin(climb_9.climb_rate / climb_8.air_speed))
    x_climb_10 = (climb_10.altitude_end - climb_9.altitude_end) / np.tan(
        np.arcsin(climb_10.climb_rate / climb_9.air_speed))
    x_descent_1 = (climb_10.altitude_end - descent_1.altitude_end) / np.tan(
        np.arcsin(descent_1.descent_rate / descent_1.air_speed))
    x_descent_2 = (descent_1.altitude_end - descent_2.altitude_end) / np.tan(
        np.arcsin(descent_2.descent_rate / descent_2.air_speed))
    x_descent_3 = (descent_2.altitude_end - descent_3.altitude_end) / np.tan(
        np.arcsin(descent_3.descent_rate / descent_3.air_speed))

    cruise_range = mission.design_range - (
                x_climb_1 + x_climb_2 + x_climb_3 + x_climb_4 + x_climb_5 + x_climb_6 + x_climb_7 + x_climb_8 +
                x_climb_9 + x_climb_10 + x_descent_1 + x_descent_2 + x_descent_3)

    segments['cruise'].distance = cruise_range

    return nexus


# ----------------------------------------------------------------------
#   Check Cruise Altitude
# ----------------------------------------------------------------------

def check_cruise_altitude(analyses):
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_dummy_mission'

    # airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude = 0.0 * Units.ft
    airport.delta_isa = 0.0
    airport.atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()

    mission.airport = airport

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    ones_row = base_segment.state.ones_row
    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position = SUAVE.Methods.skip
    base_segment.process.iterate.unknowns.network = analyses.vehicle.propulsors.turboprop.unpack_unknowns
    base_segment.process.iterate.residuals.network = analyses.vehicle.propulsors.turboprop.residuals
    base_segment.state.unknowns.pitch_command = ones_row(1) * 0. * Units.deg
    base_segment.state.conditions.propulsion = Data()
    base_segment.state.conditions.propulsion.rpm = 1200 * ones_row(1)
    base_segment.state.residuals.net = 0. * ones_row(1)
    base_segment.state.numerics.number_control_points = 10
    base_segment.state.conditions.freestream = Data()
    base_segment.state.conditions.freestream.delta_ISA = airport.delta_isa * ones_row(1)
    base_segment.state.numerics.number_control_points = 20
    planet = SUAVE.Attributes.Planets.Earth()
    atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_EAS_Constant_Rate(base_segment)
    segment.tag = "climb_dummy"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # define segment attributes
    segment.atmosphere = atmosphere
    segment.planet = planet

    segment.altitude_start       = 0.0      * Units.ft
    segment.altitude_end         = 21000.   * Units.ft
    segment.equivalent_air_speed = 168.2981 * Units.knots
    segment.climb_rate           = 300.     * Units['ft/min']
    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'
    # add to mission
    mission.append_segment(segment)

    results = mission.evaluate()

    cruise_altitude = 21000.
    for segment in results.segments.values():
        altitude = segment.conditions.freestream.altitude[:, 0] / Units.ft
        eta = segment.conditions.propulsion.throttle[:, 0]
        for i in range(len(altitude)):
            if eta[i] > 1.:
                cruise_altitude = altitude[i - 1]
            elif eta[i] < 1. and i == len(altitude) - 1:
                cruise_altitude = altitude[i]

    print ('Cruise altitude: ' + str(int((cruise_altitude+1)/100.)*100.)+' ft')
    cruise_altitude = int(cruise_altitude/100.)*100. * Units.ft

    return cruise_altitude


# ----------------------------------------------------------------------
#   Sizing
# ----------------------------------------------------------------------    

def simple_sizing(nexus):
    configs = nexus.vehicle_configurations

    HT_volume = 1.15948 * Units.less   # ATR-72
    VT_volume = 0.094316 * Units.less  # ATR-72
    lht       = 13.8654 * Units.m
    lvt       = 10.5724 * Units.m
    for config in configs:
        wing_planform(config.wings.main_wing)

        config.wings.horizontal_stabilizer.areas.reference = (HT_volume/lht)*(config.wings.main_wing.areas.reference *
                                                                              config.wings.main_wing.chords.mean_aerodynamic)
        config.wings.vertical_stabilizer.areas.reference   = (VT_volume/lvt)*(config.wings.main_wing.areas.reference *
                                                                              config.wings.main_wing.spans.projected)

        SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_fuel_volume(config.wings.main_wing)
        nexus.summary.available_fuel = config.wings.main_wing.fuel_volume * 803.0 - 3200.03305635

        for wing in config.wings:


            wing_planform(wing)
            wing.areas.wetted = 2.0 * wing.areas.reference
            wing.areas.exposed = 0.8 * wing.areas.wetted
            wing.areas.affected = 0.6 * wing.areas.wetted

    return nexus

# ----------------------------------------------------------------------        
#   Weights
# ----------------------------------------------------------------------    

def weight(nexus):
    vehicle = nexus.vehicle_configurations.base
    BOW = 13500.
    # weight analysis
    weights = nexus.analyses.base.weights.evaluate()
    weights = nexus.analyses.landing.weights.evaluate()
    weights = nexus.analyses.takeoff.weights.evaluate()
    weights = nexus.analyses.cruise.weights.evaluate()

    vehicle.mass_properties.breakdown = weights

    empty_weight     = vehicle.mass_properties.operating_empty
    passenger_weight = vehicle.passenger_weights.mass_properties.mass
    empty_weight = empty_weight - 239.

    delta = BOW - empty_weight
    vehicle.mass_properties.max_takeoff = 23000. - delta + vehicle.delta_fuel
    vehicle.mass_properties.takeoff     = vehicle.mass_properties.max_takeoff
    print ('MTOW: '+str('%5.1f' % vehicle.mass_properties.max_takeoff)+' kg, BOW: '+str('%5.1f' % empty_weight)+' kg')
    for config in nexus.vehicle_configurations:
        config.mass_properties.zero_fuel_center_of_gravity  = vehicle.mass_properties.zero_fuel_center_of_gravity
        config.fuel                                         = vehicle.fuel
        config.mass_properties.takeoff                      = vehicle.mass_properties.takeoff
        config.mass_properties.max_takeoff                  = vehicle.mass_properties.max_takeoff
    nexus.summary.MTOW = vehicle.mass_properties.max_takeoff
    nexus.summary.BOW  = empty_weight
    return nexus
# ----------------------------------------------------------------------
#   Takeoff Field Length Evaluation
# ----------------------------------------------------------------------

def takeoff_field_length(nexus):

    # import tofl analysis module
    estimate_tofl = SUAVE.Methods.Performance.estimate_take_off_field_length

    # unpack data
    results  = nexus.results
    summary  = nexus.summary
    analyses = nexus.analyses
    missions = nexus.missions
    config   = nexus.vehicle_configurations.takeoff
    vehicle  = nexus.vehicle_configurations.base

    # defining required data for tofl evaluation
    config.mass_properties.takeoff = nexus.summary.MTOW
    takeoff_airport                = missions.base.airport

    takeoff_field_length, second_segment_climb_gradient_takeoff = estimate_tofl(config, analyses, takeoff_airport, 1)

    print ('TOFL: '+str('%5.1f' % takeoff_field_length)+' m, Second Segment Climb Gradient: ' +
           str('%1.5f' % second_segment_climb_gradient_takeoff))

    # pack results
    summary.takeoff_field_length = takeoff_field_length
    summary.second_segment_climb_gradient_takeoff = second_segment_climb_gradient_takeoff

    # -----------------------------------------------------------------------------------------------------------------
    # estimating TOW for Hot and High Condition
    config = deepcopy(nexus.vehicle_configurations.takeoff)
    config.wings['main_wing'].flaps.angle = 15. * Units.deg
    config.wings['main_wing'].slats.angle =  0. * Units.deg

    takeoff_airport           = deepcopy(missions.base.airport)
    takeoff_airport.altitude  = 5333. * Units.ft
    takeoff_airport.delta_isa = 23

    TOFL_HH = [0., 0., 0., 0.]
    GRAD_HH = [0., 0., 0.]
    TOW = [21000., 21500., 22000.]
    config.mass_properties.takeoff = TOW[0] * Units.kg
    TOFL_HH[0], GRAD_HH[0] = estimate_tofl(config, analyses, takeoff_airport, 1)
    GRAD_HH[0] = float(GRAD_HH[0])
    config.mass_properties.takeoff = TOW[1] * Units.kg
    TOFL_HH[1], GRAD_HH[1] = estimate_tofl(config, analyses, takeoff_airport, 1)
    GRAD_HH[1] = float(GRAD_HH[1])
    config.mass_properties.takeoff = TOW[2] * Units.kg
    TOFL_HH[2], GRAD_HH[2] = estimate_tofl(config, analyses, takeoff_airport, 1)
    GRAD_HH[2] = float(GRAD_HH[2])

    f = interpolate.interp1d(GRAD_HH, TOW, kind='quadratic', fill_value="extrapolate")

    TOW.append(float(f(0.024)))
    GRAD_HH.append(0.)

    config.mass_properties.takeoff = TOW[3] * Units.kg
    TOFL_HH[3], GRAD_HH[3] = estimate_tofl(config, analyses, takeoff_airport, 1)
    GRAD_HH[3] = float(GRAD_HH[3])

    TOW = [x for _, x in sorted(zip(GRAD_HH, TOW))]
    GRAD_HH.sort()

    f = interpolate.interp1d(GRAD_HH, TOW, kind='quadratic', fill_value="extrapolate")

    summary.TOW_HH = float(f(0.024))

    if summary.TOW_HH > nexus.summary.MTOW:
        summary.TOW_HH = nexus.summary.MTOW
        print('Warning: TOW for Hot and High TO > MTOW')
        # check_hh_range(nexus)
    else:
        print('TOW for Hot and High: ' + str('%5.1f' % summary.TOW_HH))
        # check_hh_range(nexus)

    return nexus

# ----------------------------------------------------------------------
#   landing Field Length Evaluation
# ----------------------------------------------------------------------

def landing_field_length(nexus):

    # import tofl analysis module
    estimate_landing = SUAVE.Methods.Performance.estimate_landing_field_length

    # unpack data
    summary = nexus.summary
    analyses = nexus.analyses
    missions = nexus.missions
    config   = nexus.vehicle_configurations.landing

    # defining required data for tofl evaluation
    landing_airport = missions.base.airport
    config.mass_properties.landing = nexus.summary.MTOW * 0.97174

    landing_field_length = estimate_landing(config, analyses, landing_airport)

    print ('LFL for MLW (' + str('%5.1f' % config.mass_properties.landing) + ' kg): ' +
           str('%5.1f' % landing_field_length) + ' m')

    # pack results
    summary.landing_field_length = landing_field_length

    return nexus

# ----------------------------------------------------------------------
#   Check range for hot and high TOW
# ----------------------------------------------------------------------

def check_hh_range(nexus):
    mission = deepcopy(nexus.missions.base)
    vehicle = nexus.vehicle_configurations.base

    mission.tag = 'Range_for_HH'
    mission.airport.altitude = 5333. * Units.ft
    mission.segments['climb_1'].altitude_start = 5333. * Units.ft

    cruise_segment_tag = 'cruise'

    # Define takeoff weight
    mission.segments[0].analyses.weights.vehicle.mass_properties.takeoff = nexus.summary.TOW_HH

    # Evaluate mission with current TOW
    results = mission.evaluate()
    segment = results.segments[cruise_segment_tag]

    maxIter = 10  # maximum iteration limit
    tol = 1.  # fuel convergency tolerance
    err = 9999.  # error to be minimized
    iter = 0  # iteration count

    BOW = vehicle.mass_properties.operating_empty
    TOW = nexus.summary.TOW_HH
    FUEL = TOW - 5000. - BOW

    while abs(err) > tol and iter < maxIter:
        iter = iter + 1

        # Current total fuel burned in mission
        TotalFuel = TOW - results.segments[-1].conditions.weights.total_mass[-1, 0]

        # Difference between burned fuel and target fuel
        missingFuel = FUEL - TotalFuel

        # Current distance and fuel consumption in the cruise segment
        CruiseDist = np.diff(segment.conditions.frames.inertial.position_vector[[0, -1], 0])[0]  # Distance [m]
        CruiseFuel = segment.conditions.weights.total_mass[0, 0] - segment.conditions.weights.total_mass[-1, 0]  # [kg]
        # Current specific range (m/kg)
        CruiseSR = CruiseDist / CruiseFuel  # [m/kg]

        # Estimated distance that will result in total fuel burn = target fuel
        DeltaDist = CruiseSR * missingFuel
        mission.segments[cruise_segment_tag].distance = (CruiseDist + DeltaDist)

        # running mission with new distance
        results = mission.evaluate()
        segment = results.segments[cruise_segment_tag]

        # Difference between burned fuel and target fuel
        err = (TOW - results.segments[-1].conditions.weights.total_mass[-1, 0]) - FUEL

    if iter >= maxIter:
        print 'Warning: Range for Hot and High did not converge for given fuel'

    nexus.summary.range_HH = results.segments['descent_3'].conditions.frames.inertial.position_vector[-1, 0] * (
            Units.m / Units.nautical_mile)
    print ('Range for Hot and High TO (' + str('%5.1f' % TOW) + '): ' + str('%5.1f' % nexus.summary.range_HH))

    return nexus


# ----------------------------------------------------------------------
#   Finalizing Function
# ----------------------------------------------------------------------    

def finalize(nexus):
    
    nexus.analyses.finalize()   
    
    return nexus         

# ----------------------------------------------------------------------
#   Post Process Results to give back to the optimizer
# ----------------------------------------------------------------------   

def post_process(nexus):
    
    # Unpack data
    vehicle                           = nexus.vehicle_configurations.base
    results                           = nexus.results
    summary                           = nexus.summary
    missions                          = nexus.missions  
    nexus.total_number_of_iterations +=1
    # Static stability calculations
    CMA = -10.
    for segment in results.base.segments.values():
        max_CMA = np.max(segment.conditions.stability.static.cm_alpha[:, 0])
        if max_CMA > CMA:
            CMA = max_CMA
            
    summary.static_stability = CMA
    
    #throttle in design mission
    max_throttle = 0.
    for segment in results.base.segments.values():
        max_segment_throttle = np.max(segment.conditions.propulsion.throttle[:, 0])
        if max_segment_throttle > max_throttle:
            max_throttle = max_segment_throttle
    min_throttle = 1.
    for segment in results.base.segments.values():
        min_segment_throttle = np.min(segment.conditions.propulsion.throttle[:, 0])
        if min_segment_throttle < min_throttle:
            min_throttle = min_segment_throttle

    summary.throttle_max = 1.00001 - max_throttle  # strategy to make constraint > 0
    summary.throttle_min = min_throttle + 0.01

    # Fuel margin and base fuel calculations
    operating_empty          = summary.BOW
    max_payload              = vehicle.mass_properties.max_payload
    design_landing_weight    = results.base.segments[-1].conditions.weights.total_mass[-1]
    design_takeoff_weight    = results.base.segments[0].conditions.weights.total_mass[0]
    zero_fuel_weight         = vehicle.mass_properties.max_zero_fuel

    summary.base_mission_fuelburn = design_takeoff_weight - design_landing_weight
    summary.fuel_margin_ub        = design_landing_weight - operating_empty - max_payload
    summary.fuel_margin_lb        = 10. - (design_landing_weight - operating_empty - max_payload)
    print('Fuel Margin: ' + str('%4.1f' % summary.fuel_margin_ub) + ' kg')
    summary.mzfw_consistency      = zero_fuel_weight - operating_empty - max_payload

    # ------------------------------------------------------------------------------------------------------------------
    # Additional Constraints computation
    # TOFL
    summary.takeoff_field_length_margin = 1391. - summary.takeoff_field_length

    if summary.takeoff_field_length_margin > -0.05 and summary.takeoff_field_length_margin < 0:
        summary.takeoff_field_length_margin = - summary.takeoff_field_length_margin

    # Second Segment Climb Gradient
    summary.climb_gradient = summary.second_segment_climb_gradient_takeoff - 0.03

    # summary.range_HH_margin = summary.range_HH - 1216.8
    summary.TOW_HH_margin = (summary.TOW_HH - 6400. - operating_empty) - 1100.
    summary.FUEL_HH       = summary.TOW_HH - 6400. - operating_empty
    print('Fuel available - HH: ' + str('%4.1f' % (summary.TOW_HH - 6400. - operating_empty)) + ' kg')

    # Time to climb
    time_f = results.base.segments['climb_10'].conditions.frames.inertial.time[-1] / Units.min
    time_i = results.base.segments['climb_1'].conditions.frames.inertial.time[0] / Units.min
    summary.time_to_climb = 34.9 - (time_f - time_i)
    summary.time_to_climb_value = (time_f - time_i)

    print ('Time to climb to cruise altitude: ' + str('%2.1f' % (time_f - time_i)) + ' min')

    # Design Range
    summary.cruise_range = -((results.base.segments['climb_5'].conditions.frames.inertial.position_vector[-1, 0] *
                                       Units.m/Units.nautical_mile) -
                            (results.base.segments['cruise'].conditions.frames.inertial.position_vector[-1, 0] *
                                       Units.m/Units.nautical_mile))
    summary.design_range = (results.base.segments['descent_3'].conditions.frames.inertial.position_vector[-1, 0] *
                                       Units.m/Units.nautical_mile)
    summary.total_range  = (results.base.segments[-1].conditions.frames.inertial.position_vector[-1, 0] *
                                       Units.m/Units.nautical_mile)
    summary.design_range_margin = 500.0 - (results.base.segments['descent_3'].conditions.frames.inertial.position_vector[-1, 0] *
                                       Units.m/Units.nautical_mile)
    summary.design_range_ub = 500.2 - (results.base.segments['descent_3'].conditions.frames.inertial.position_vector[-1, 0] *
                                       Units.m/Units.nautical_mile)
    summary.design_range_lb = -499.8 + (results.base.segments['descent_3'].conditions.frames.inertial.position_vector[-1, 0] *
                                       Units.m/Units.nautical_mile)

    if summary.design_range_ub > -0.05 and summary.design_range_ub < 0:
        summary.design_range_ub = - summary.design_range_ub

    if summary.design_range_lb > -0.05 and summary.design_range_lb < 0:
        summary.design_range_lb = - summary.design_range_lb

    print('Design Range: ' + str('%4.1f' % (results.base.segments['descent_3'].conditions.frames.inertial.position_vector[-1, 0] *
                                       Units.m/Units.nautical_mile)))

    summary.block_fuel = design_takeoff_weight - results.base.segments['descent_3'].conditions.weights.total_mass[-1, 0]
    summary.cruise_fuel = (results.base.segments['climb_5'].conditions.weights.total_mass[-1, 0] -
                           results.base.segments['cruise'].conditions.weights.total_mass[-1, 0])
    # LFL
    summary.lfl_mlw_margin = 917. - summary.landing_field_length

    summary.max_fuel_margin = summary.available_fuel - 5000.
    print ('Maximum Fuel available: ' + str('%4.1f' % summary.available_fuel) + ' kg')
    print ('Fuel Burn: ' + str('%4.1f' % summary.base_mission_fuelburn) + ' kg')
    print('MTOW: ' + str('%5.1f' % summary.MTOW))
    # beta = vehicle.wings['main_wing'].beta
    # summary.objective = summary.base_mission_fuelburn * beta + summary.MTOW/4. * (1-beta)

    # print('Beta: '+str('%1.1f' % beta)+' , Objective: ' + str('%1.4f' % summary.objective))
    summary.nothing = 0.
    #when you run want to output results to a file
    # filename = 'results.txt'
    # write_optimization_outputs(nexus, filename)
    print('\n')
    return nexus    
