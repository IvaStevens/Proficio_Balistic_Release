from ctypes import *



MAX_CONTROL_DIMS = 18 # Variable c_int '18'
MAX_EM_DIMS = MAX_CONTROL_DIMS # alias
def check_flag_bits(A,bitmask): return (((A) & (bitmask)) == (bitmask)) # macro
# def set_flag_bits(A,bitmask): return ((A) |= (bitmask)) # macro
# def clear_flag_bits(A,bitmask): return ((A) &= ~(bitmask)) # macro
STRING_DATA = c_char * 0
class MDF_TRIAL_EVENT(Structure):
    pass
MDF_TRIAL_EVENT._fields_ = [
    ('event_time', c_double),
    ('event_code', c_int),
    ('reserved', c_int),
]
class MDF_TRIAL_CONFIG(Structure):
    pass
MDF_TRIAL_CONFIG._fields_ = [
    ('rep_no', c_int),
    ('trial_no', c_int),
    ('num_reward_states', c_int),
    ('reserved', c_int),
]
class MDF_TRIAL_STATUS(Structure):
    pass
MDF_TRIAL_STATUS._fields_ = [
    ('success', c_int),
    ('reserved', c_int),
]
class ROBOT_ACTUAL_STATE(Structure):
    pass
MDF_ROBOT_ACTUAL_STATE = ROBOT_ACTUAL_STATE
class ROBOT_CONTROL_STATE(Structure):
    pass
MDF_ROBOT_CONTROL_STATE = ROBOT_CONTROL_STATE
class ROBOT_CONTROL_SPACE_ACTUAL_STATE(Structure):
    pass
MDF_ROBOT_CONTROL_SPACE_ACTUAL_STATE = ROBOT_CONTROL_SPACE_ACTUAL_STATE
class ROBOT_CONTROL_CONFIG(Structure):
    pass
MDF_ROBOT_CONTROL_CONFIG = ROBOT_CONTROL_CONFIG
class MDF_GIVE_REWARD(Structure):
    pass
MDF_GIVE_REWARD._fields_ = [
    ('duration_ms', c_double),
    ('num_clicks', c_double),
]
class N19MDF_IO_START_STREAM4DOT_76E(Structure):
    pass
N19MDF_IO_START_STREAM4DOT_76E._fields_ = [
    ('high', c_double * 12),
    ('low', c_double * 12),
]
class MDF_IO_START_STREAM(Structure):
    pass
MDF_IO_START_STREAM._fields_ = [
    ('limits', N19MDF_IO_START_STREAM4DOT_76E),
    ('reserved', c_int),
    ('internal_sampling', c_int),
    ('sample_interval', c_double),
]
class MDF_ANALOG_STREAM(Structure):
    pass
class SAMPLE_HEADER(Structure):
    pass
SAMPLE_HEADER._fields_ = [
    ('SerialNo', c_int),
    ('Flags', c_int),
    ('DeltaTime', c_double),
]
MDF_ANALOG_STREAM._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('sample_interval', c_double),
    ('data', c_double * 12),
]
class MDF_DENSO_MOVE(Structure):
    pass
MDF_DENSO_MOVE._fields_ = [
    ('position_no', c_int),
    ('movement_id', c_int),
]
class MDF_DENSO_SET_SPEED(Structure):
    pass
MDF_DENSO_SET_SPEED._fields_ = [
    ('speed', c_double),
]
MDF_DENSO_MOVE_CONTINUE = MDF_DENSO_MOVE
MDF_DENSO_MOVE_STARTED = MDF_DENSO_MOVE
MDF_DENSO_NOT_READY = STRING_DATA
class RESPONSE_DATA_TO_DENSO_MOVE(Structure):
    pass
RESPONSE_DATA_TO_DENSO_MOVE._fields_ = [
    ('movement_id', c_int),
    ('reserved', c_int),
]
MDF_DENSO_MOVE_COMPLETE = RESPONSE_DATA_TO_DENSO_MOVE
MDF_DENSO_MOVE_FAILED = RESPONSE_DATA_TO_DENSO_MOVE
MDF_DENSO_MOVE_INVALID = RESPONSE_DATA_TO_DENSO_MOVE
MDF_DENSO_HALTED = RESPONSE_DATA_TO_DENSO_MOVE
class MOVEMENT_COMMAND_DATA(Structure):
    pass
MDF_EM_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
MDF_EM_AUX_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
MDF_COMPOSITE_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
MDF_ROBOT_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
MDF_PLANNER_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
class PLANNER_STATE(Structure):
    pass
MDF_PLANNER_STATE = PLANNER_STATE
class PLAN_PROCESSOR_STATE(Structure):
    pass
MDF_PLAN_PROCESSOR_STATE = PLAN_PROCESSOR_STATE
MDF_FM_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
class OVERRIDE_COMMAND(Structure):
    pass
MDF_OVERRIDE_COMMAND = OVERRIDE_COMMAND
class JVEL_COMMAND(Structure):
    pass
MDF_JVEL_COMMAND = JVEL_COMMAND
MDF_COMPONENT_DELAY_MSG_TYPES = c_int * 6
class DELAY_DATA(Structure):
    pass
DELAY_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('Components', c_double * 6),
    ('Total', c_double),
]
MDF_ESTIMATED_DELAY = DELAY_DATA
MDF_MEASURED_DELAY = DELAY_DATA
MDF_CLEAR_FUNCTION = c_char * 0
class TARG_CLOUD(Structure):
    pass
TARG_CLOUD._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('cloud', c_double * 576),
    ('nTargs', c_int),
    ('spacer', c_int),
]
MDF_TARG_CLOUD = TARG_CLOUD
MDF_ATTENTION = c_double
MOVEMENT_COMMAND_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('sample_interval', c_double),
    ('pos', c_double * 18),
    ('vel', c_double * 18),
    ('controlledDims', c_int * 18),
    ('tag', c_char * 64),
]
class INPUT_DOF_DATA(Structure):
    pass
INPUT_DOF_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('dof_vals', c_double * 21),
    ('tag', c_char * 64),
]
class TRIAL_EVENT(Structure):
    pass
TRIAL_EVENT._fields_ = [
    ('event_time', c_double),
    ('event_code', c_int),
    ('reserved', c_int),
]
PLAN_PROCESSOR_STATE._fields_ = [
    ('D', c_double * 576),
]
class AUTOMAGIC_CONTROLLER_PARAMS(Structure):
    pass
AUTOMAGIC_CONTROLLER_PARAMS._fields_ = [
    ('virtualPadPosition', c_double * 18),
    ('centralPointPosition', c_double * 3),
    ('posMin', c_double * 18),
    ('posMax', c_double * 18),
    ('velMin', c_double * 18),
    ('velMax', c_double * 18),
    ('graspingGraspApertureTarget', c_double),
    ('graspingOpenApertureTarget', c_double),
    ('graspingIntermediateApertureTarget', c_double),
    ('maxApproachSwingAngle', c_double),
    ('graspTooCloseIfClosedRadius', c_double),
]
ROBOT_CONTROL_CONFIG._fields_ = [
    ('target', c_double * 18),
    ('coriMatrix', c_double * 9),
    ('trialEvent', TRIAL_EVENT),
    ('autoControlParams', AUTOMAGIC_CONTROLLER_PARAMS),
    ('autoPosControlFraction', c_double * 18),
    ('autoVelControlFraction', c_double * 18),
    ('orthPosImpedance', c_double * 18),
    ('orthVelImpedance', c_double * 18),
    ('extrinsicVelControlled', c_int * 18),
    ('extrinsicPosControlled', c_int * 18),
    ('importantDOF', c_int * 18),
]
ROBOT_ACTUAL_STATE._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('RTMA_received_time', c_double),
    ('Jpos', c_double * 7),
    ('Jvel', c_double * 7),
    ('Cpos', c_double * 3),
    ('Cori', c_double * 3),
    ('Cforce', c_double * 3),
    ('Ctrq', c_double * 3),
    ('Cvel', c_double * 3),
    ('CangVel', c_double * 3),
    ('Hpos', c_double * 4),
    ('Hstrain', c_double * 4),
    ('Jtrq', c_double * 3),
]
ROBOT_CONTROL_SPACE_ACTUAL_STATE._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('RTMA_received_time', c_double),
    ('pos', c_double * 18),
    ('vel', c_double * 18),
    ('CoriMatrix', c_double * 9),
]
PLANNER_STATE._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('targetPos', c_double * 18),
]
JVEL_COMMAND._fields_ = [
    ('cmd', c_double * 11),
]
OVERRIDE_COMMAND._fields_ = [
    ('DOFCommand', c_double * 18),
    ('JposCommand', c_double * 7),
    ('controllerId', c_int),
    ('j0nks', c_int),
]
ROBOT_CONTROL_STATE._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('autoControl', PLANNER_STATE),
    ('overrideCommand', OVERRIDE_COMMAND),
    ('RTMA_received_time', c_double),
    ('actualControlPos', c_double * 18),
    ('actualControlVel', c_double * 18),
    ('autoControlPos', c_double * 18),
    ('autoControlVel', c_double * 18),
    ('autoPosControlFraction', c_double * 18),
    ('autoVelControlFraction', c_double * 18),
    ('orthPosImpedance', c_double * 18),
    ('orthVelImpedance', c_double * 18),
    ('intrinsicVelControlled', c_int * 18),
    ('intrinsicPosControlled', c_int * 18),
    ('extrinsicVelControlled', c_int * 18),
    ('extrinsicPosControlled', c_int * 18),
    ('overrideDimensions', c_int * 18),
    ('actualCommandComposition', c_uint),
    ('blank', c_int),
]
MDF_INPUT_DOF_DATA = INPUT_DOF_DATA
MDF_OPERATOR_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
MDF_FIXTURED_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
MDF_SHADOW_COMPOSITE_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
MDF_FIXTURED_COMPOSITE_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
class MDF_PROBOT_FEEDBACK(Structure):
    pass
MDF_PROBOT_FEEDBACK._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('tool_pos', c_double * 7),
    ('wrist_pos', c_double * 7),
]
class MDF_GROBOT_COMMAND(Structure):
    pass
MDF_GROBOT_COMMAND._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('command', c_double * 18),
    ('mode', c_int),
    ('reserved', c_int),
]
class MDF_GROBOT_BYPASS(Structure):
    pass
MDF_GROBOT_BYPASS._fields_ = [
    ('command', c_double * 18),
    ('mode', c_int),
    ('reserved', c_int),
]
class MDF_OPTO_CNTRL_CMD(Structure):
    pass
MDF_OPTO_CNTRL_CMD._fields_ = [
    ('Cmd', c_double * 6),
]
class MDF_OPTO_POS_CMD(Structure):
    pass
MDF_OPTO_POS_CMD._fields_ = [
    ('Cmd', c_double * 4),
]
class MDF_KIN_POS_CMD(Structure):
    pass
MDF_KIN_POS_CMD._fields_ = [
    ('Cmd', c_double * 4),
]
class MDF_KINECT_SKELETON(Structure):
    pass
MDF_KINECT_SKELETON._fields_ = [
    ('x', c_double * 20),
    ('y', c_double * 20),
    ('z', c_double * 20),
    ('w', c_double * 20),
    ('Which', c_int),
    ('Reserved', c_int),
]
class MDF_GROBOT_SEGMENT_PERCEPTS(Structure):
    pass
MDF_GROBOT_SEGMENT_PERCEPTS._fields_ = [
    ('ind_force', c_double * 3),
    ('mid_force', c_double * 3),
    ('rng_force', c_double * 3),
    ('lit_force', c_double * 3),
    ('thb_force', c_double * 3),
    ('ind_accel', c_double * 3),
    ('mid_accel', c_double * 3),
    ('rng_accel', c_double * 3),
    ('lit_accel', c_double * 3),
    ('thb_accel', c_double * 3),
]
class MDF_CERESTIM_CONFIG_MODULE(Structure):
    pass
MDF_CERESTIM_CONFIG_MODULE._fields_ = [
    ('configID', c_int),
    ('afcf', c_int),
    ('pulses', c_double),
    ('amp1', c_double),
    ('amp2', c_double),
    ('width1', c_double),
    ('width2', c_double),
    ('frequency', c_double),
    ('interphase', c_double),
]
class MDF_CERESTIM_CONFIG_CHAN(Structure):
    pass
MDF_CERESTIM_CONFIG_CHAN._fields_ = [
    ('stop', c_int),
    ('group_stimulus', c_int),
    ('group_numChans', c_int),
    ('group_channel', c_int * 16),
    ('group_pattern', c_int * 16),
    ('manual_stimulus', c_int),
    ('manual_channel', c_int),
    ('manual_pattern', c_int),
]
class MDF_GROBOT_RAW_FEEDBACK(Structure):
    pass
MDF_GROBOT_RAW_FEEDBACK._fields_ = [
    ('j_ang', c_double * 28),
    ('j_vel', c_double * 28),
    ('j_trq', c_double * 28),
]
class MDF_GROBOT_FEEDBACK(Structure):
    pass
MDF_GROBOT_FEEDBACK._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('position', c_double * 18),
    ('velocity', c_double * 18),
    ('force', c_double * 18),
    ('cori_matrix', c_double * 9),
]
class MDF_SESSION_CONFIG(Structure):
    pass
MDF_SESSION_CONFIG._fields_ = [
    ('data_dir', c_char * 128),
]
class MDF_GLOVE_DATA(Structure):
    pass
MDF_GLOVE_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('data', c_double * 30),
]
class MDF_MICROSTRAIN_DATA(Structure):
    pass
MDF_MICROSTRAIN_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('data', c_double * 30),
]
class MDF_EM_OVERRIDE_CONFIG(Structure):
    pass
MDF_EM_OVERRIDE_CONFIG._fields_ = [
    ('chosen_channel_mask', c_int * 1728),
]
class MDF_CHANGE_TOOL(Structure):
    pass
MDF_CHANGE_TOOL._fields_ = [
    ('next_tool_id', c_int),
    ('reserved', c_int),
]
class MDF_CHANGE_TOOL_COMPLETE(Structure):
    pass
MDF_CHANGE_TOOL_COMPLETE._fields_ = [
    ('tool_id', c_int),
    ('reserved', c_int),
]
class MDF_BURT_STATUS(Structure):
    pass
MDF_BURT_STATUS._fields_ = [
    ('error', c_bool),
    ('error_msg', c_char * 64),
    ('task_complete', c_bool),
    ('task_success', c_bool),
    ('timestamp', c_ulonglong),
    ('user_force', c_double),
    ('pos_x', c_double),
    ('pos_y', c_double),
    ('pos_z', c_double),
    ('force_x', c_double),
    ('force_y', c_double),
    ('force_z', c_double),
    ('state', c_int),
]
class MDF_TASK_STATE_CONFIG(Structure):
    pass
MDF_TASK_STATE_CONFIG._fields_ = [
    ('state', c_int),
    ('force', c_double),
    ('distance', c_double),
    ('target_width', c_double),
    ('id', c_int),
    ('rep_num', c_int),
    ('use_for_calib', c_int),
    ('target_combo_index', c_int),
    ('timed_out_conseq', c_int),
    ('reach_offset', c_int),
    ('relax_arm', c_int),
    ('idle_gateable', c_int),
    ('force_gateable', c_int),
    ('direction', c_int),
    ('idle_timeout', c_double),
    ('ts_time', c_double),
    ('target', c_double * 33),
    ('coriMatrix', c_double * 9),
    ('idle_target', c_double * 18),
    ('trans_threshold', c_double),
    ('ori_threshold', c_double),
    ('trans_threshold_f', c_double),
    ('ori_threshold_f', c_double),
    ('sep_threshold', c_double * 12),
    ('sep_threshold_f', c_double * 12),
    ('sep_threshold_judging_polarity', c_int * 12),
    ('sep_threshold_f_judging_polarity', c_int * 12),
    ('sep_threshold_judging_outcome', c_int * 12),
    ('trans_threshold_judging_polarity', c_int),
    ('ori_threshold_judging_polarity', c_int),
    ('trans_threshold_f_judging_polarity', c_int),
    ('ori_threshold_f_judging_polarity', c_int),
    ('handle_judging_polarity', c_int),
    ('handle_judging_outcome', c_int),
    ('timeout', c_double),
    ('tags', c_char * 64),
    ('fdbk_display_color', c_char * 64),
    ('background_color', c_char * 64),
]
class MDF_JUDGE_VERDICT(Structure):
    pass
MDF_JUDGE_VERDICT._fields_ = [
    ('id', c_int),
    ('reserved', c_int),
    ('reason', c_char * 64),
]
class MDF_END_TASK_STATE(Structure):
    pass
MDF_END_TASK_STATE._fields_ = [
    ('id', c_int),
    ('outcome', c_int),
    ('reason', c_char * 64),
]
class MDF_RAW_SAMPLE_RESPONSE(Structure):
    pass
MDF_RAW_SAMPLE_RESPONSE._fields_ = [
    ('source_index', c_int),
    ('reserved', c_int),
    ('source_timestamp', c_double),
]
class MDF_CODE_VERSION(Structure):
    pass
MDF_CODE_VERSION._fields_ = [
    ('module_name', c_char * 64),
    ('version', c_char * 64),
]
class MDF_SPM_FIRINGRATE(Structure):
    pass
MDF_SPM_FIRINGRATE._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('source_timestamp', c_double),
    ('count_interval', c_double),
    ('rates', c_double * 1728),
]
MDF_EM_DECODER_CONFIGURATION = c_ubyte * 0
class MDF_LOAD_DECODER_CONFIG(Structure):
    pass
MDF_LOAD_DECODER_CONFIG._fields_ = [
    ('full_path', c_char * 128),
]
class MDF_XM_START_SESSION(Structure):
    pass
MDF_XM_START_SESSION._fields_ = [
    ('load_calibration', c_int),
    ('calib_session_id', c_int),
    ('num_reps', c_int),
    ('reserved', c_int),
    ('subject_name', c_char * 64),
]
class MDF_PING(Structure):
    pass
MDF_PING._fields_ = [
    ('module_name', c_char * 64),
]
class MDF_PING_ACK(Structure):
    pass
MDF_PING_ACK._fields_ = [
    ('module_name', c_char * 64),
]
class MDF_DEBUG_VECTOR(Structure):
    pass
MDF_DEBUG_VECTOR._fields_ = [
    ('data', c_double * 32),
]
class MDF_APP_START(Structure):
    pass
MDF_APP_START._fields_ = [
    ('config', c_char * 64),
]
class MDF_MODULE_START(Structure):
    pass
MDF_MODULE_START._fields_ = [
    ('module', c_char * 64),
]
class MDF_SPM_SPIKE_SNIPPET(Structure):
    pass
MDF_SPM_SPIKE_SNIPPET._fields_ = [
    ('time', c_double),
    ('chan', c_int),
    ('unit', c_int),
    ('box_id', c_int),
    ('length', c_int),
    ('snippet', c_short * 48),
]
class MDF_SPM_SPIKE_TIMES(Structure):
    pass
MDF_SPM_SPIKE_TIMES._fields_ = [
    ('time', c_double * 256),
    ('chan', c_short * 256),
    ('unit', c_char * 256),
    ('box_id', c_char * 256),
]
class MDF_IDLY_LABELLING(Structure):
    pass
MDF_IDLY_LABELLING._fields_ = [
    ('state', c_int),
    ('reserved', c_int),
]
class MDF_EM_DRIFT_CORRECTION(Structure):
    pass
MDF_EM_DRIFT_CORRECTION._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('drift_correction', c_double * 18),
]
class MDF_PLAY_SOUND(Structure):
    pass
MDF_PLAY_SOUND._fields_ = [
    ('id', c_int),
    ('reserved', c_int),
]
class MDF_ARTIFACT_REJECTED(Structure):
    pass
MDF_ARTIFACT_REJECTED._fields_ = [
    ('time', c_double),
]
class MDF_IDLE(Structure):
    pass
MDF_IDLE._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('score', c_double),
    ('gain', c_double),
    ('idle', c_int),
    ('reserved', c_int),
]
MDF_WAM_FEEDBACK = ROBOT_CONTROL_SPACE_ACTUAL_STATE
MDF_WAM_HAND_FEEDBACK = ROBOT_CONTROL_SPACE_ACTUAL_STATE
class MDF_PLANNER_CONTROL_CONFIG(Structure):
    pass
MDF_PLANNER_CONTROL_CONFIG._fields_ = [
    ('target', c_double * 18),
    ('coriMatrix', c_double * 9),
]
class MDF_ROBOT_JOINT_COMMAND(Structure):
    pass
MDF_ROBOT_JOINT_COMMAND._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('pos', c_double * 8),
    ('controlledDims', c_int * 8),
    ('overrideDims', c_int * 8),
]
MDF_IDLEGATED_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
class MDF_OUTPUT_DOF_DATA(Structure):
    pass
MDF_OUTPUT_DOF_DATA._fields_ = [
    ('dof_vals', c_double * 21),
    ('tag', c_char * 64),
]
class MDF_FORCE_APPLIED(Structure):
    pass
MDF_FORCE_APPLIED._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('applied', c_int),
    ('reserved', c_int),
]
class MDF_RAW_FORCE_SENSOR_DATA(Structure):
    pass
MDF_RAW_FORCE_SENSOR_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('rdt_sequence', c_uint),
    ('ft_sequence', c_uint),
    ('status', c_uint),
    ('reserved', c_uint),
    ('data', c_double * 6),
]
class MDF_FORCE_SENSOR_DATA(Structure):
    pass
MDF_FORCE_SENSOR_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('rdt_sequence', c_uint),
    ('ft_sequence', c_uint),
    ('status', c_uint),
    ('reserved', c_uint),
    ('data', c_double * 6),
    ('offset', c_double * 6),
]
class MDF_SYNC_PULSE(Structure):
    pass
MDF_SYNC_PULSE._fields_ = [
    ('SerialNo', c_uint),
    ('reserved', c_uint),
]
class MDF_MUSCLE(Structure):
    pass
MDF_MUSCLE._fields_ = [
    ('channelNum', c_uint),
    ('reserved', c_uint),
    ('name', c_char * 64),
]
class MDF_COMBO_WAIT(Structure):
    pass
MDF_COMBO_WAIT._fields_ = [
    ('duration', c_uint),
    ('reserved', c_uint),
]
class MDF_FSR(Structure):
    pass
MDF_FSR._fields_ = [
    ('channelNum', c_uint),
    ('reserved', c_uint),
    ('name', c_char * 64),
]
class MDF_FSR_DATA(Structure):
    pass
MDF_FSR_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('data', c_double * 8),
]
class MDF_FORCE_FEEDBACK(Structure):
    pass
MDF_FORCE_FEEDBACK._fields_ = [
    ('x', c_double),
    ('y', c_double),
    ('z', c_double),
]
class MDF_POSITION_FEEDBACK(Structure):
    pass
MDF_POSITION_FEEDBACK._fields_ = [
    ('x', c_double),
    ('y', c_double),
    ('z', c_double),
]
class MDF_TRIAL_INPUT(Structure):
    pass
MDF_TRIAL_INPUT._fields_ = [
    ('XorYorZ', c_int),
    ('UpOrDown', c_int),
    ('forceThreshold', c_double),
    ('targetDistance', c_double),
    ('targetError', c_double),
    ('direction', c_double),
    ('width', c_double),
]
class MDF_RT_POSITION_FEEDBACK(Structure):
    pass
MDF_RT_POSITION_FEEDBACK._fields_ = [
    ('distanceFromCenter', c_double),
]
class MDF_TRIAL_STATUS_FEEDBACK(Structure):
    pass
MDF_TRIAL_STATUS_FEEDBACK._fields_ = [
    ('trialComplete', c_int),
    ('reserved', c_int),
]
class MDF_RAW_SPIKECOUNT(Structure):
    pass
MDF_RAW_SPIKECOUNT._fields_ = [
    ('source_index', c_int),
    ('reserved', c_int),
    ('source_timestamp', c_double),
    ('count_interval', c_double),
    ('counts', c_ubyte * 576),
]
SPIKE_COUNT_DATA_TYPE = c_ubyte
class MDF_SPM_SPIKECOUNT(Structure):
    pass
MDF_SPM_SPIKECOUNT._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('source_timestamp', c_double),
    ('count_interval', c_double),
    ('counts', SPIKE_COUNT_DATA_TYPE * 1728),
]
class MDF_SAMPLE_GENERATED(Structure):
    pass
MDF_SAMPLE_GENERATED._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('source_timestamp', c_double),
]
class MDF_SAMPLE_RESPONSE(Structure):
    pass
MDF_SAMPLE_RESPONSE._fields_ = [
    ('sample_response_timestamp', c_double),
    ('sample_alignment_timestamp', c_double),
    ('sample_response_count', c_uint),
    ('sample_alignment_count', c_uint),
]
MT_EM_MOVEMENT_COMMAND = 350 # Variable c_int '350'
MID_GATING_JUDGE = 43 # Variable c_int '43'
MPL_AT_ALL_JP = 4 # Variable c_int '4'
MT_READY_BUTTON = 614 # Variable c_int '614'
SF_IRREGULAR = 4 # Variable c_int '4'
MT_ENABLE_DATA_COLLECTION = 2380 # Variable c_int '2380'
MT_JUDGE_VERDICT = 1960 # Variable c_int '1960'
MID_ARDUINO_IO_WRITE_TESTER = 41 # Variable c_int '41'
MID_OPTO_MPL_CTRL = 32 # Variable c_int '32'
MT_SYNCH_DONE = 1802 # Variable c_int '1802'
MAX_OPTO_CTRL_JOINTS = 6 # Variable c_int '6'
MT_IDLY_LABELLING = 2190 # Variable c_int '2190'
MID_SILICON_MONKEY = 72 # Variable c_int '72'
MT_RESUME_EXPERIMENT = 181 # Variable c_int '181'
MT_DENSO_HALT = 803 # Variable c_int '803'
MT_APP_START_COMPLETE = 2121 # Variable c_int '2121'
MT_APP_START = 2120 # Variable c_int '2120'
MID_SIMPLE_ARBITRATOR = 13 # Variable c_int '13'
MID_SIMPLE_PLANNER = 76 # Variable c_int '76'
MT_XM_END_OF_SESSION = 2180 # Variable c_int '2180'
NUM_COMPONENT_DELAYS = 6 # Variable c_int '6'
MT_OVERRIDE_COMMAND = 480 # Variable c_int '480'
ASC_MONKEY_KNOB = 0 # Variable c_int '0'
MID_DIGITAL_IO = 17 # Variable c_int '17'
MT_EM_DECODER_CONFIGURATION = 2090 # Variable c_int '2090'
MID_EM_OVERRIDE_CONFIG = 31 # Variable c_int '31'
MID_NETBOX_MODULE = 25 # Variable c_int '25'
MAX_PROBOT_FEEDBACK_DIMS = 7 # Variable c_int '7'
MT_FIXTURED_MOVEMENT_COMMAND = 1910 # Variable c_int '1910'
MAX_CYBER_GLOVE_DIMS = 30 # Variable c_int '30'
MT_SPM_FIRINGRATE = 2100 # Variable c_int '2100'
MT_CHANGE_TOOL_FAILED = 1943 # Variable c_int '1943'
SF_FRACTINT = 8 # Variable c_int '8'
MT_TRIAL_STATUS = 102 # Variable c_int '102'
MT_CERESTIM_CONFIG_MODULE = 1889 # Variable c_int '1889'
MT_DEBUG_VECTOR = 2160 # Variable c_int '2160'
MT_IO_STOP_STREAM = 602 # Variable c_int '602'
MID_ARDUINO_IO_READ_TESTER = 40 # Variable c_int '40'
MT_OUTPUT_DOF_DATA = 2310 # Variable c_int '2310'
MT_TRIAL_EVENT = 100 # Variable c_int '100'
MID_SPM_MOD4 = 24 # Variable c_int '24'
MT_MICROSTRAIN_DATA = 1705 # Variable c_int '1705'
MT_GRASP_EVENT = 937 # Variable c_int '937'
MID_SPM_MOD1 = 21 # Variable c_int '21'
MID_SPM_MOD2 = 22 # Variable c_int '22'
MID_SPM_MOD3 = 23 # Variable c_int '23'
MT_RELOAD_CONFIGURATION = 1004 # Variable c_int '1004'
MT_DENSO_WIGGLE = 806 # Variable c_int '806'
MID_GROBOT_FEEDBACK = 18 # Variable c_int '18'
MT_IDLEGATED_MOVEMENT_COMMAND = 2280 # Variable c_int '2280'
MT_CHANGE_TOOL_COMPLETE = 1942 # Variable c_int '1942'
MT_KIN_POS_CMD = 1713 # Variable c_int '1713'
MT_PLAN_PROCESSOR_STATE = 354 # Variable c_int '354'
MT_JOYPAD_PAD_BUTTON = 613 # Variable c_int '613'
MT_DENSO_HALTED = 855 # Variable c_int '855'
MT_SYNCH_START = 1801 # Variable c_int '1801'
MID_SURFACE_EMG = 36 # Variable c_int '36'
MAX_FILENAME_LENGTH = 256 # Variable c_int '256'
MT_ENABLE_SYNC_PULSE = 2400 # Variable c_int '2400'
MID_IO_MOD = 60 # Variable c_int '60'
MT_LATE_ADAPT_NOW = 1005 # Variable c_int '1005'
MT_END_TASK_STATE = 1970 # Variable c_int '1970'
MID_SSH_CONTROLLER = 35 # Variable c_int '35'
MT_BURT_STATUS = 1269 # Variable c_int '1950'
MT_TASK_STATE_CONFIG = 1950 # Variable c_int '1950'
MT_CLEAR_FUNCTION = 1003 # Variable c_int '1003'
MT_DENSO_INITIALIZE = 801 # Variable c_int '801'
MAX_OPTO_POS = 4 # Variable c_int '4'
MT_SPM_SPIKECOUNT = 1751 # Variable c_int '1751'
MT_SESSION_CONFIG = 1710 # Variable c_int '1710'
MID_FEEDBACK_TRANSFORM = 11 # Variable c_int '11'
MID_COLOR_CUE = 46 # Variable c_int '46'
MID_SILICON_MONKEY_BRAIN = 91 # Variable c_int '91'
MID_FSR_EMG = 47 # Variable c_int '47'
MT_EXIT_ACK = 2130 # Variable c_int '2130'
MT_IO_STREAM_STARTED = 621 # Variable c_int '621'
MID_CERESTIM_CONFIG = 57 # Variable c_int '57'
MID_DENSO_TRY = 81 # Variable c_int '81'
MAX_UNITS_PER_CHAN = 6 # Variable c_int '6'
MT_INPUT_DOF_DATA = 1850 # Variable c_int '1850'
ASC_ROBOT_DOOR = 3 # Variable c_int '3'
MID_PS3_COMMAND_MODULE = 95 # Variable c_int '95'
MT_CHANGE_TOOL_INVALID = 1941 # Variable c_int '1941'
MT_FORCE_SENSOR_DATA = 2330 # Variable c_int '2330'
MPL_AT_ARM_EPV_FING_JV = 0 # Variable c_int '0'
MT_FIXTURED_COMPOSITE_MOVEMENT_COMMAND = 1920 # Variable c_int '1920'
MT_JOYPAD_X = 612 # Variable c_int '612'
MT_EM_OVERRIDE_CONFIG = 1708 # Variable c_int '1708'
MPL_AT_ARM_EPV_FING_JP = 1 # Variable c_int '1'
SAMPLE_DT = 0.03 # Variable c_double '2.99999999999999988897769753748434595763683319091796875e-2'
MT_KINECT_SKELETON = 1711 # Variable c_int '1711'
MAX_SPIKE_TIMES_PER_PACKET = 256 # Variable c_int '256'
MT_OPTO_CNTRL_CMD = 1709 # Variable c_int '1709'
MT_WAM_HAND_FEEDBACK = 2242 # Variable c_int '2242'
NUM_FINGER_DIMS = 10 # Variable c_int '10'
MT_RAW_FORCE_SENSOR_DATA = 2340 # Variable c_int '2340'
MID_VIRTUAL_FIXTURING = 28 # Variable c_int '28'
MT_RAW_SAMPLE_RESPONSE = 1980 # Variable c_int '1980'
MT_IO_START_STREAM = 601 # Variable c_int '601'
MT_POSITION_FEEDBACK = 2423 # Variable c_int '2423'
MID_OPTOTRAK = 74 # Variable c_int '74'
MT_DENSO_READY = 850 # Variable c_int '850'
MT_EM_DRIFT_CORRECTION = 2210 # Variable c_int '2210'
MAX_TOTAL_SPIKE_CHANS = 1728 # Variable c_int '1728'
MT_ROBOT_ACTUAL_STATE = 934 # Variable c_int '934'
ASC_JOYPAD_LEFT_UD = 8 # Variable c_int '8'
MT_OPERATOR_MOVEMENT_COMMAND = 1900 # Variable c_int '1900'
MAX_PERCEPT_DIMS = 15 # Variable c_int '15'
MID_FSR_READ = 45 # Variable c_int '45'
MT_SPM_SPIKE_TIMES = 2171 # Variable c_int '2171'
MPL_AT_ALL_JV = 3 # Variable c_int '3'
ASC_JOYPAD_LEFT_LR = 9 # Variable c_int '9'
MAX_JOINT_DIMS = 8 # Variable c_int '8'
MT_MEASURED_DELAY = 202 # Variable c_int '202'
MT_FSR = 2420 # Variable c_int '2420'
MT_IDLE = 2250 # Variable c_int '2250'
MT_PLAY_SOUND = 2230 # Variable c_int '2230'
MT_IDLY_RESET_LABELLING = 2200 # Variable c_int '2200'
MT_MODULE_START = 2125 # Variable c_int '2125'
MT_SAMPLE_GENERATED = 1752 # Variable c_int '1752'
MAX_SEPARATE_DIMS = 12 # Variable c_int '12'
MID_COMMAND_SPACE_FEEDBACK_GUI = 89 # Variable c_int '89'
MID_HANDY_DANDY = 86 # Variable c_int '86'
MT_ATTENTION = 1659 # Variable c_int '1659'
MT_ANALOG_STREAM = 620 # Variable c_int '620'
MAX_KIN_POS = 4 # Variable c_int '4'
MT_FORCE_FEEDBACK = 2422 # Variable c_int '2422'
MT_DENSO_MOVE_COMPLETE = 852 # Variable c_int '852'
MID_TEST_MOD = 99 # Variable c_int '99'
ASC_ARMREST_LF = 4 # Variable c_int '4'
MT_ESTIMATED_DELAY = 201 # Variable c_int '201'
MAX_EM_CHANNELS = 1728 # Variable c_int '1728'
MAX_GROBOT_COMMAND_DIMS = 18 # Variable c_int '18'
MT_GROBOT_RAW_FEEDBACK = 1701 # Variable c_int '1701'
MT_PING_ACK = 2141 # Variable c_int '2141'
MID_TASK_JUDGE = 16 # Variable c_int '16'
MT_GROBOT_FEEDBACK = 1702 # Variable c_int '1702'
MT_FM_MOVEMENT_COMMAND = 430 # Variable c_int '430'
MT_FSR_DATA = 2421 # Variable c_int '2421'
MT_GIVE_REWARD = 600 # Variable c_int '600'
MAX_TOTAL_SPIKE_CHANS_PER_SOURCE = 576 # Variable c_int '576'
MAX_SPIKE_CHANS_PER_SOURCE = 96 # Variable c_int '96'
MT_DENSO_CONFIG = 800 # Variable c_int '800'
HID_MAIN_HOST = 1 # Variable c_int '1'
MT_TRIAL_DATA_SAVED = 2080 # Variable c_int '2080'
MT_IO_STREAM_STOPPED = 622 # Variable c_int '622'
MAX_FINGER_DIMS = 10 # Variable c_int '10'
SF_UNFREEZE = 2 # Variable c_int '2'
MSEC_PER_RAW_SAMPLE = 10 # Variable c_int '10'
MID_DENSO_GATE = 15 # Variable c_int '15'
MT_GLOVE_DATA = 1706 # Variable c_int '1706'
MT_DUMMY_MESSAGE = 2000 # Variable c_int '2000'
MAX_DOFS = 21 # Variable c_int '21'
MID_MESSAGE_WATCHER = 88 # Variable c_int '88'
MT_WAM_FEEDBACK = 2240 # Variable c_int '2240'
MT_PING = 2140 # Variable c_int '2140'
MID_VIZ = 85 # Variable c_int '85'
ASC_ARMREST_LB = 5 # Variable c_int '5'
MT_ROBOT_CONTROL_CONFIG = 938 # Variable c_int '938'
MID_EXTRACTION_MOD = 50 # Variable c_int '50'
ASC_ROBOT_KNOB = 2 # Variable c_int '2'
MT_ROBOT_MOVEMENT_COMMAND = 351 # Variable c_int '351'
MID_GROBOT_RAW_FEEDBACK = 19 # Variable c_int '19'
RAW_COUNTS_PER_SAMPLE = 3 # Variable c_int '3'
MT_DENSO_MOVE_STARTED = 810 # Variable c_int '810'
MID_WAM = 73 # Variable c_int '73'
MID_EXEC_MOD = 10 # Variable c_int '10'
MT_DENSO_MOVE_FAILED = 853 # Variable c_int '853'
ASC_MONKEY_DOOR = 1 # Variable c_int '1'
MID_INPUT_TRANSFORM = 14 # Variable c_int '14'
MT_FORCE_APPLIED = 2320 # Variable c_int '2320'
MID_HAND_VIZ = 30 # Variable c_int '30'
MT_PROBOT_FEEDBACK = 1930 # Variable c_int '1930'
MT_RESET_SAMPLE_ALIGNMENT = 1754 # Variable c_int '1754'
MT_GROBOT_END_BYPASS = 1707 # Variable c_int '1707'
MID_OBSTACLE_COURSE = 33 # Variable c_int '33'
MT_DENSO_NOT_READY = 851 # Variable c_int '851'
MT_MODULE_START_COMPLETE = 2126 # Variable c_int '2126'
MT_SHADOW_COMPOSITE_MOVEMENT_COMMAND = 1919 # Variable c_int '1919'
MT_DENSO_MOVE_INVALID = 854 # Variable c_int '854'
MT_PLANNER_MOVEMENT_COMMAND = 352 # Variable c_int '352'
MT_SYNCH_NOW = 1800 # Variable c_int '1800'
MID_ARDUINO_MODULE = 42 # Variable c_int '42'
MT_DENSO_MOVE = 802 # Variable c_int '802'
MT_EM_AUX_MOVEMENT_COMMAND = 355 # Variable c_int '355'
MID_PS3_RAW_MODULE = 96 # Variable c_int '96'
MT_PLANNER_STATE = 353 # Variable c_int '353'
TAG_LENGTH = 64 # Variable c_int '64'
KINECT_JOINTS = 20 # Variable c_int '20'
MID_GROBOT_SEGMENT_PERCEPTS = 56 # Variable c_int '56'
ASC_JOYPAD_RIGHT_UD = 10 # Variable c_int '10'
MAX_HAND_DIMS = 25 # Variable c_int '25'
MID_SPM_MOD = 20 # Variable c_int '20'
MT_ROBOT_CONTROL_SPACE_ACTUAL_STATE = 936 # Variable c_int '936'
MT_DATA_COLLECTED = 2370 # Variable c_int '2370'
MT_ARTIFACT_REJECTED = 2220 # Variable c_int '2220'
MID_CALIBRATION = 37 # Variable c_int '37'
MT_PLANNER_CONTROL_CONFIG = 2260 # Variable c_int '2260'
MT_TARG_CLOUD = 1006 # Variable c_int '1006'
MT_PROCEED_TO_Failure = 196 # Variable c_int '196'
SF_ALIGNMENT = 1 # Variable c_int '1'
MT_KEYBOARD = 1002 # Variable c_int '1002'
NUM_D_COLS = 16 # Variable c_int '16'
MT_START_PAD_PRESSED = 2290 # Variable c_int '2290'
MID_ARDUINO_IO = 39 # Variable c_int '39'
MT_COMPOSITE_MOVEMENT_COMMAND = 366 # Variable c_int '366'
MT_RT_POSITION_FEEDBACK = 2425 # Variable c_int '2425'
MT_COMBO_WAIT = 2410 # Variable c_int '2410'
MPL_AT_ARM_JV_FING_JP = 2 # Variable c_int '2'
MT_PAUSE_EXPERIMENT = 180 # Variable c_int '180'
MT_STOP_DATA_COLLECT = 2365 # Variable c_int '2365'
MT_START_DATA_COLLECT = 2360 # Variable c_int '2360'
MPL_AT_ARM_EPP_FING_JP = 5 # Variable c_int '5'
MT_KILL_KILL_KILL_EM = 300 # Variable c_int '300'
MT_JOYPAD_R2 = 611 # Variable c_int '611'
MT_CERESTIM_CONFIG_CHAN = 1890 # Variable c_int '1890'
MT_COMMANDSPACE_FEEDBACK = 1703 # Variable c_int '1703'
MAX_DATA_DIR_LEN = 128 # Variable c_int '128'
MT_TRIAL_STATUS_FEEDBACK = 2426 # Variable c_int '2426'
MT_JOYPAD_R1 = 610 # Variable c_int '610'
MT_JVEL_COMMAND = 481 # Variable c_int '481'
MT_ROBOT_CONTROL_STATE = 935 # Variable c_int '935'
MT_MUSCLE = 2390 # Variable c_int '2390'
ASC_ARMREST_RB = 7 # Variable c_int '7'
MT_DATA_COLLECT_STARTED = 2362 # Variable c_int '2362'
MAX_D_COLS = 32 # Variable c_int '32'
ASC_ARMREST_RF = 6 # Variable c_int '6'
MT_CODE_VERSION = 1990 # Variable c_int '1990'
MT_TRIAL_CONFIG = 101 # Variable c_int '101'
MT_SPM_SPIKE_SNIPPET = 2170 # Variable c_int '2170'
MT_GROBOT_SEGMENT_PERCEPTS = 1888 # Variable c_int '1888'
MID_CERESTIM_CONTROL = 58 # Variable c_int '58'
MID_WATCHDOG = 97 # Variable c_int '97'
MID_DRIFTY = 93 # Variable c_int '93'
MT_XM_ABORT_SESSION = 171 # Variable c_int '171'
MT_ROBOT_JOINT_COMMAND = 2270 # Variable c_int '2270'
MID_ARDUINO_SYNC = 44 # Variable c_int '44'
MT_START_PAD_RELEASED = 2300 # Variable c_int '2300'
MT_NOOP = 998 # Variable c_int '998'
MT_XM_START_SESSION = 2150 # Variable c_int '2150'
ASC_JOYPAD_RIGHT_LR = 11 # Variable c_int '11'
MT_GROBOT_COMMAND = 1700 # Variable c_int '1700'
MT_IDLE_DETECTION_ENDED = 2251 # Variable c_int '2251'
MID_OUTPUT_TRANSFORM = 12 # Variable c_int '12'
NUM_ANALOG_STREAM_CHANS = 12 # Variable c_int '12'
MT_CHANGE_TOOL = 1940 # Variable c_int '1940'
MT_DENSO_SET_SPEED = 804 # Variable c_int '804'
MT_PROCEED_TO_NextState = 198 # Variable c_int '198'
MID_SKELETON_CTRL = 34 # Variable c_int '34'
MT_DENSO_MOVE_CONTINUE = 807 # Variable c_int '807'
MAX_GROBOT_FEEDBACK_DIMS = 18 # Variable c_int '18'
MT_RAW_SPIKECOUNT = 1750 # Variable c_int '1750'
MT_COMPONENT_DELAY_MSG_TYPES = 200 # Variable c_int '200'
MID_SAMPLE_GENERATOR = 29 # Variable c_int '29'
MID_CANNED_MOVEMENT = 27 # Variable c_int '27'
MAX_SPIKE_SOURCES = 3 # Variable c_int '3'
MAX_GROBOT_JOINTS = 28 # Variable c_int '28'
MID_THE_DECIDER = 94 # Variable c_int '94'
MT_GROBOT_BYPASS = 1704 # Variable c_int '1704'
MT_TRIAL_INPUT = 2424 # Variable c_int '2424'
MID_PS3_MODULE = 92 # Variable c_int '92'
MID_DENSO_MOD = 80 # Variable c_int '80'
MT_OPTO_POS_CMD = 1712 # Variable c_int '1712'
MT_SAMPLE_RESPONSE = 1753 # Variable c_int '1753'
MID_CUBE_SPHERE = 38 # Variable c_int '38'
MT_LOAD_DECODER_CONFIG = 2110 # Variable c_int '2110'
MT_SYNC_PULSE = 2350 # Variable c_int '2350'
__all__ = ['MT_EM_MOVEMENT_COMMAND', 'MT_OVERRIDE_COMMAND',
           'MDF_DENSO_MOVE', 'MT_JUDGE_VERDICT',
           'MDF_IO_START_STREAM', 'MT_SYNCH_DONE',
           'MDF_GROBOT_RAW_FEEDBACK', 'MT_DENSO_SET_SPEED',
           'MT_DENSO_HALT', 'MT_APP_START', 'MT_XM_END_OF_SESSION',
           'MDF_SHADOW_COMPOSITE_MOVEMENT_COMMAND', 'INPUT_DOF_DATA',
           'N19MDF_IO_START_STREAM4DOT_76E', 'MID_NETBOX_MODULE',
           'MT_FIXTURED_MOVEMENT_COMMAND', 'MDF_SPM_SPIKE_SNIPPET',
           'MT_DEBUG_VECTOR', 'MDF_PLANNER_STATE', 'MID_SPM_MOD4',
           'MDF_IDLE', 'MID_SPM_MOD1', 'MID_SPM_MOD2', 'MID_SPM_MOD3',
           'MT_DENSO_WIGGLE', 'MID_GROBOT_FEEDBACK', 'MT_KIN_POS_CMD',
           'MT_DENSO_HALTED', 'MDF_ANALOG_STREAM',
           'MT_TASK_STATE_CONFIG', 'MT_BURT_STATUS', 'MT_DENSO_INITIALIZE',
           'NUM_ANALOG_STREAM_CHANS', 'STRING_DATA', 'MDF_MUSCLE',
           'MDF_CERESTIM_CONFIG_CHAN', 'MPL_AT_ARM_EPV_FING_JV',
           'MT_FIXTURED_COMPOSITE_MOVEMENT_COMMAND',
           'SPIKE_COUNT_DATA_TYPE', 'MPL_AT_ARM_EPV_FING_JP',
           'MT_IDLY_LABELLING', 'ROBOT_CONTROL_CONFIG',
           'MID_VIRTUAL_FIXTURING', 'MT_POSITION_FEEDBACK',
           'MDF_PROBOT_FEEDBACK', 'MDF_FM_MOVEMENT_COMMAND',
           'MDF_WAM_FEEDBACK', 'MID_FSR_READ', 'MPL_AT_ALL_JV',
           'MPL_AT_ALL_JP', 'MT_IDLE', 'MT_IDLY_RESET_LABELLING',
           'MDF_CHANGE_TOOL', 'MAX_SEPARATE_DIMS', 'MID_HANDY_DANDY',
           'MT_ATTENTION', 'MDF_CODE_VERSION', 'MDF_CLEAR_FUNCTION',
           'MDF_TASK_STATE_CONFIG', 'MDF_BURT_STATUS', 'MT_TRIAL_INPUT',
           'MAX_GROBOT_COMMAND_DIMS', 'MID_EM_OVERRIDE_CONFIG',
           'MID_TEST_MOD', 'MT_DENSO_CONFIG', 'MT_TRIAL_DATA_SAVED',
           'MDF_GIVE_REWARD', 'MDF_FSR', 'MT_WAM_HAND_FEEDBACK',
           'MID_DENSO_GATE', 'MDF_IDLEGATED_MOVEMENT_COMMAND',
           'MT_DUMMY_MESSAGE', 'MID_VIZ', 'MT_DENSO_MOVE_STARTED',
           'MID_WAM', 'MT_START_PAD_PRESSED', 'ASC_MONKEY_DOOR',
           'MT_PROBOT_FEEDBACK', 'MDF_DENSO_MOVE_FAILED',
           'MID_OBSTACLE_COURSE', 'MDF_PING_ACK', 'MT_DENSO_MOVE',
           'MID_FEEDBACK_TRANSFORM', 'ROBOT_ACTUAL_STATE',
           'ASC_JOYPAD_RIGHT_UD', 'MDF_APP_START',
           'MDF_POSITION_FEEDBACK', 'MT_PLANNER_CONTROL_CONFIG',
           'SF_ALIGNMENT', 'MDF_WAM_HAND_FEEDBACK', 'MID_EXEC_MOD',
           'MT_STOP_DATA_COLLECT', 'MPL_AT_ARM_EPP_FING_JP',
           'MT_KILL_KILL_KILL_EM', 'MDF_ROBOT_JOINT_COMMAND',
           'MDF_FORCE_SENSOR_DATA', 'MDF_KINECT_SKELETON',
           'ASC_ARMREST_RB', 'MT_IO_STOP_STREAM',
           'MID_CERESTIM_CONTROL', 'MID_WATCHDOG',
           'MT_ROBOT_JOINT_COMMAND', 'MDF_OPERATOR_MOVEMENT_COMMAND',
           'MDF_ATTENTION', 'MT_DENSO_MOVE_CONTINUE',
           'MT_RAW_SPIKECOUNT', 'MDF_ROBOT_MOVEMENT_COMMAND',
           'MT_SAMPLE_RESPONSE', 'MID_GATING_JUDGE',
           'MID_SIMPLE_ARBITRATOR', 'check_flag_bits',
           'MT_EM_DECODER_CONFIGURATION', 'MAX_CYBER_GLOVE_DIMS',
           'MT_SPM_FIRINGRATE', 'SF_FRACTINT',
           'MT_CERESTIM_CONFIG_MODULE', 'ASC_ARMREST_RF', 'MT_NOOP',
           'MDF_ROBOT_CONTROL_SPACE_ACTUAL_STATE',
           'MT_ENABLE_SYNC_PULSE', 'MID_IO_MOD', 'MT_LATE_ADAPT_NOW',
           'MT_END_TASK_STATE', 'MID_SSH_CONTROLLER',
           'MT_GROBOT_SEGMENT_PERCEPTS', 'MT_CLEAR_FUNCTION',
           'TARG_CLOUD', 'MT_SPM_SPIKECOUNT', 'MT_EXIT_ACK',
           'MDF_MODULE_START', 'MT_FORCE_SENSOR_DATA', 'MT_SYNCH_NOW',
           'MID_SAMPLE_GENERATOR', 'MDF_RT_POSITION_FEEDBACK',
           'MT_RAW_SAMPLE_RESPONSE', 'MAX_TOTAL_SPIKE_CHANS',
           'MDF_EM_DECODER_CONFIGURATION', 'MT_SPM_SPIKE_TIMES',
           'MT_DENSO_NOT_READY', 'MDF_PLANNER_CONTROL_CONFIG',
           'ASC_JOYPAD_LEFT_UD', 'TRIAL_EVENT',
           'MDF_XM_START_SESSION', 'MT_DENSO_MOVE_COMPLETE',
           'MDF_COMBO_WAIT', 'MT_ANALOG_STREAM',
           'MDF_PLAN_PROCESSOR_STATE', 'MID_TASK_JUDGE',
           'MT_FM_MOVEMENT_COMMAND', 'MT_FSR_DATA',
           'MDF_FORCE_FEEDBACK', 'HID_MAIN_HOST',
           'MT_RAW_FORCE_SENSOR_DATA', 'MT_WAM_FEEDBACK',
           'ASC_ARMREST_LF', 'ASC_ARMREST_LB', 'NUM_COMPONENT_DELAYS',
           'DELAY_DATA', 'MDF_COMPOSITE_MOVEMENT_COMMAND',
           'MT_ROBOT_MOVEMENT_COMMAND', 'MDF_SYNC_PULSE',
           'MT_IDLE_DETECTION_ENDED', 'MDF_SAMPLE_RESPONSE',
           'MT_GROBOT_END_BYPASS', 'MT_PLANNER_STATE',
           'MDF_FIXTURED_MOVEMENT_COMMAND', 'MDF_SESSION_CONFIG',
           'MT_INPUT_DOF_DATA', 'MID_CERESTIM_CONFIG',
           'MAX_HAND_DIMS', 'MDF_PLAY_SOUND', 'MID_SPM_MOD',
           'MDF_DEBUG_VECTOR', 'MDF_FORCE_APPLIED',
           'MID_PS3_COMMAND_MODULE', 'MAX_KIN_POS', 'MID_PS3_MODULE',
           'MAX_D_COLS', 'MDF_TRIAL_CONFIG',
           'MDF_ROBOT_CONTROL_CONFIG', 'MT_OPTO_POS_CMD',
           'MID_ARDUINO_SYNC', 'MDF_GROBOT_BYPASS',
           'MDF_MICROSTRAIN_DATA', 'ASC_ROBOT_DOOR',
           'MT_PROCEED_TO_NextState', 'MID_EXTRACTION_MOD',
           'MDF_TARG_CLOUD', 'MT_EM_OVERRIDE_CONFIG',
           'MDF_SPM_FIRINGRATE', 'MDF_DENSO_MOVE_STARTED',
           'MDF_GROBOT_COMMAND', 'SF_IRREGULAR', 'MT_SYNC_PULSE',
           'MID_OPTO_MPL_CTRL', 'MDF_DENSO_SET_SPEED',
           'MID_SILICON_MONKEY', 'SAMPLE_HEADER',
           'MT_APP_START_COMPLETE', 'MID_SIMPLE_PLANNER',
           'MDF_OUTPUT_DOF_DATA', 'MID_GROBOT_RAW_FEEDBACK',
           'MAX_UNITS_PER_CHAN', 'ROBOT_CONTROL_STATE',
           'MT_CHANGE_TOOL_FAILED', 'MDF_EM_OVERRIDE_CONFIG',
           'MID_ARDUINO_IO_READ_TESTER', 'MT_TRIAL_EVENT',
           'MT_GRASP_EVENT', 'MAX_EM_DIMS', 'MT_RELOAD_CONFIGURATION',
           'MT_CHANGE_TOOL_COMPLETE', 'MT_SYNCH_START',
           'MAX_FILENAME_LENGTH', 'MT_SESSION_CONFIG',
           'MID_SILICON_MONKEY_BRAIN', 'MDF_KIN_POS_CMD',
           'MID_DENSO_TRY', 'MDF_MEASURED_DELAY',
           'MAX_SPIKE_TIMES_PER_PACKET', 'MT_OPTO_CNTRL_CMD',
           'MT_CERESTIM_CONFIG_CHAN', 'MID_FSR_EMG',
           'MT_ROBOT_ACTUAL_STATE', 'MT_OPERATOR_MOVEMENT_COMMAND',
           'MAX_JOINT_DIMS', 'MT_MEASURED_DELAY', 'MT_FSR',
           'MDF_EM_DRIFT_CORRECTION', 'MT_PLAY_SOUND',
           'MID_ARDUINO_MODULE', 'MT_DENSO_READY',
           'MDF_DENSO_MOVE_INVALID', 'MDF_RAW_FORCE_SENSOR_DATA',
           'MDF_COMPONENT_DELAY_MSG_TYPES',
           'MAX_SPIKE_CHANS_PER_SOURCE', 'MID_SURFACE_EMG',
           'MDF_END_TASK_STATE', 'MSEC_PER_RAW_SAMPLE', 'MAX_DOFS',
           'MID_MESSAGE_WATCHER', 'MT_PING_ACK',
           'MDF_ARTIFACT_REJECTED', 'PLANNER_STATE',
           'MID_OUTPUT_TRANSFORM', 'MDF_OPTO_CNTRL_CMD',
           'MDF_FIXTURED_COMPOSITE_MOVEMENT_COMMAND',
           'MDF_ROBOT_CONTROL_STATE', 'MDF_EM_MOVEMENT_COMMAND',
           'MT_TRIAL_STATUS_FEEDBACK', 'MID_INPUT_TRANSFORM',
           'MDF_DENSO_MOVE_CONTINUE', 'MT_RESET_SAMPLE_ALIGNMENT',
           'MDF_FSR_DATA', 'MT_MODULE_START_COMPLETE',
           'MT_DENSO_MOVE_INVALID', 'MDF_JUDGE_VERDICT',
           'MT_MODULE_START', 'MDF_ESTIMATED_DELAY',
           'MT_MICROSTRAIN_DATA', 'TAG_LENGTH',
           'MID_GROBOT_SEGMENT_PERCEPTS',
           'MT_ROBOT_CONTROL_SPACE_ACTUAL_STATE',
           'MT_ARTIFACT_REJECTED', 'MDF_OVERRIDE_COMMAND',
           'MT_KEYBOARD', 'MID_COLOR_CUE',
           'RESPONSE_DATA_TO_DENSO_MOVE', 'MDF_TRIAL_STATUS',
           'MID_ARDUINO_IO', 'MT_PAUSE_EXPERIMENT',
           'MT_START_DATA_COLLECT', 'MDF_DENSO_MOVE_COMPLETE',
           'AUTOMAGIC_CONTROLLER_PARAMS', 'MDF_RAW_SPIKECOUNT',
           'MAX_DATA_DIR_LEN', 'MDF_SPM_SPIKE_TIMES', 'MT_MUSCLE',
           'MT_CODE_VERSION', 'MT_TRIAL_CONFIG',
           'MOVEMENT_COMMAND_DATA', 'MT_GROBOT_RAW_FEEDBACK',
           'MT_XM_ABORT_SESSION', 'ROBOT_CONTROL_SPACE_ACTUAL_STATE',
           'MDF_LOAD_DECODER_CONFIG', 'ASC_JOYPAD_RIGHT_LR',
           'MT_CHANGE_TOOL', 'MAX_GROBOT_FEEDBACK_DIMS',
           'MID_CANNED_MOVEMENT', 'KINECT_JOINTS', 'MT_GROBOT_BYPASS',
           'MDF_RAW_SAMPLE_RESPONSE', 'MDF_JVEL_COMMAND',
           'JVEL_COMMAND', 'MAX_GROBOT_JOINTS', 'MID_CUBE_SPHERE',
           'MT_READY_BUTTON', 'MT_ENABLE_DATA_COLLECTION',
           'MID_ARDUINO_IO_WRITE_TESTER', 'MDF_TRIAL_STATUS_FEEDBACK',
           'MAX_OPTO_CTRL_JOINTS', 'MT_RESUME_EXPERIMENT',
           'ASC_ROBOT_KNOB', 'MID_DIGITAL_IO',
           'MT_COMPOSITE_MOVEMENT_COMMAND', 'MT_ROBOT_CONTROL_STATE',
           'MAX_TOTAL_SPIKE_CHANS_PER_SOURCE', 'MT_OUTPUT_DOF_DATA',
           'MT_IDLEGATED_MOVEMENT_COMMAND', 'MT_PLAN_PROCESSOR_STATE',
           'MT_JOYPAD_PAD_BUTTON', 'MT_ROBOT_CONTROL_CONFIG',
           'MDF_PING', 'PLAN_PROCESSOR_STATE', 'MAX_OPTO_POS',
           'MT_IO_STREAM_STOPPED', 'MDF_DENSO_NOT_READY',
           'MDF_TRIAL_EVENT', 'MT_CHANGE_TOOL_INVALID',
           'MDF_GROBOT_SEGMENT_PERCEPTS', 'OVERRIDE_COMMAND',
           'MAX_PERCEPT_DIMS', 'MDF_OPTO_POS_CMD', 'SAMPLE_DT',
           'MT_KINECT_SKELETON', 'MDF_GROBOT_FEEDBACK',
           'NUM_FINGER_DIMS', 'MT_IO_START_STREAM', 'MID_OPTOTRAK',
           'MT_EM_DRIFT_CORRECTION', 'MDF_TRIAL_INPUT',
           'MAX_PROBOT_FEEDBACK_DIMS', 'ASC_JOYPAD_LEFT_LR',
           'MDF_INPUT_DOF_DATA', 'MDF_DENSO_HALTED',
           'MT_SAMPLE_GENERATED', 'MT_FORCE_APPLIED',
           'MAX_CONTROL_DIMS', 'MT_FORCE_FEEDBACK',
           'MDF_SAMPLE_GENERATED', 'MT_START_PAD_RELEASED',
           'MT_ESTIMATED_DELAY', 'MAX_EM_CHANNELS',
           'MDF_CERESTIM_CONFIG_MODULE', 'MT_GROBOT_FEEDBACK',
           'MDF_ROBOT_ACTUAL_STATE', 'MAX_FINGER_DIMS', 'SF_UNFREEZE',
           'MT_GLOVE_DATA', 'MT_PING', 'MDF_SPM_SPIKECOUNT',
           'MID_HAND_VIZ', 'MT_SHADOW_COMPOSITE_MOVEMENT_COMMAND',
           'MT_PLANNER_MOVEMENT_COMMAND', 'MID_PS3_RAW_MODULE',
           'MDF_IDLY_LABELLING', 'MT_DATA_COLLECTED',
           'MID_CALIBRATION', 'MDF_EM_AUX_MOVEMENT_COMMAND',
           'MT_TARG_CLOUD', 'MT_PROCEED_TO_Failure', 'NUM_D_COLS',
           'MT_IO_STREAM_STARTED', 'MT_RT_POSITION_FEEDBACK',
           'MT_COMBO_WAIT', 'MPL_AT_ARM_JV_FING_JP', 'MT_JOYPAD_R1',
           'MT_JOYPAD_R2', 'MT_DENSO_MOVE_FAILED',
           'MT_COMMANDSPACE_FEEDBACK', 'MT_JVEL_COMMAND',
           'ASC_MONKEY_KNOB', 'MT_DATA_COLLECT_STARTED',
           'MT_SPM_SPIKE_SNIPPET', 'MID_DRIFTY',
           'MT_XM_START_SESSION', 'MT_GROBOT_COMMAND',
           'MT_GIVE_REWARD', 'MID_SKELETON_CTRL',
           'RAW_COUNTS_PER_SAMPLE', 'MT_TRIAL_STATUS', 'MT_JOYPAD_X',
           'MT_COMPONENT_DELAY_MSG_TYPES',
           'MT_EM_AUX_MOVEMENT_COMMAND', 'MDF_GLOVE_DATA',
           'MAX_SPIKE_SOURCES', 'MID_COMMAND_SPACE_FEEDBACK_GUI',
           'MID_THE_DECIDER', 'MDF_CHANGE_TOOL_COMPLETE',
           'MID_DENSO_MOD', 'MT_LOAD_DECODER_CONFIG',
           'MDF_PLANNER_MOVEMENT_COMMAND']
