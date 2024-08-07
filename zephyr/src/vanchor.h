
typedef struct {
	char type[4];
	int param1;
	int param2;
	int param3;
	int param4;
	int param5;
} vanchor_msg_t;

enum vanchor_cmd_type {
	VANCHOR_NONE = -1,
	VANCHOR_UPD = 0,
	VANCHOR_CAL,
	VANCHOR_PING,
	VANCHOR_NUM_TYPE
};

typedef struct {

	/*
	 * The distance
	 * from the current position
	 * to the target position.
	 */
	int32_t step_dis2go;

	/* desired stepper motor values */
	int32_t des_step_pos;
	int32_t des_step_speed;
	int32_t des_step_acc;

	/* desired dc motor values */
	int32_t des_dc_speed;
	bool des_dc_rev;

	/* current stepper motor values */
	int32_t cur_step_pos;
	int32_t cur_step_speed;
	int32_t cur_step_acc;

	/* current dc motor values */
	int32_t cur_dc_speed;
	bool cur_dc_rev;

	/* calibration */
	int32_t calib_begin;
	int32_t calib_end;

} vanchor_motors_handler_t;