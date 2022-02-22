#ifndef CAVR_SYSTEM_UTILS_HPP_
#define CAVR_SYSTEM_UTILS_HPP_

namespace cavr {
	// Vehicle status (flag)
	uint32_t PAUSE = pow(2, 0);
	uint32_t STOP = pow(2, 1);
	uint32_t INTER_ACTIVE = pow(2, 2);
	uint32_t MANUAL_OVERRIDE = pow(2, 3);
	uint32_t MANUAL_OVERRIDE_POSSIBLE_RESET = pow(2, 4);
	uint32_t LOG = pow(2, 5);
	uint32_t DYNPOS = pow(2, 6);
	uint32_t ENABLE_USBL_DVL = pow(2, 7);
	uint32_t EXEC_VELCMD = pow(2, 8);
	uint32_t EXEC_ALTCMD = pow(2, 9);
	uint32_t EXEC_DEPTHCMD = pow(2, 10);
	uint32_t AUTONOMOUS = pow(2, 11);
	uint32_t EXEC_PATH_FORWARD = pow(2, 12);
	uint32_t EXEC_PATH_REVERSE = pow(2, 13);
	uint32_t EXEC_PATH_LEFT = pow(2, 14);
	uint32_t EXEC_PATH_RIGHT = pow(2, 15);
	uint32_t END_OF_MISSION = pow(2, 16);
	uint32_t HOME = pow(2, 17);
	uint32_t EXEC_ALTCMD_FILE = pow(2, 18);
	uint32_t EXEC_DEPTHCMD_FILE = pow(2, 19);

	// Define system-wide requests
	// use 0-199 for system wide
	uint8_t GOAL_CONTINUE = 1;
	uint8_t MISSION_RESET = 2;
	uint8_t MISSION_LOAD_FILE = 3;
	uint8_t MISSION_SAVE_FILE = 4;
	uint8_t MISSION_STAR = 5;
	uint8_t MISSION_LAWN_N = 6;
	uint8_t MISSION_LAWN_E = 7;
	uint8_t MISSION_LAWN_S = 8;
	uint8_t MISSION_LAWN_W = 9;
	uint8_t MISSION_VEH = 10;
	uint8_t MISSION_BOX = 11;
	uint8_t MAP_SET_VEH_POS = 12;

	// use 200-255 for vehicle specific
	uint8_t GSS_CAL_DEPTH = 200;
	uint8_t GSS_SET_VEH_POS = 201;
	uint8_t GSS_SET_MAG_DECL = 202;

	// Define control modes available
	uint8_t CONTROLLER_NONE = 0;
	uint8_t CONTROLLER_TRAVERSE = 1;
	uint8_t CONTROLLER_TELE_OP = 2;
	uint8_t CONTROLLER_DYN_POS = 3;
	uint8_t CONTROLLER_STATION_KEEP = 4;
	uint8_t CONTROLLER_HOVER = 5;
    uint8_t CONTROLLER_HOVER_AND_TRANSLATE = 6;
}

#endif // CAVR_SYSTEM_UTILS_HPP_



