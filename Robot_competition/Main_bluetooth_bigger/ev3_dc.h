
/*  ev3_dc.h was generated by yup.py (yupp) 1.0b4
    out of ev3_dc.yu-h 
 *//**
 *  \file  ev3_dc.h (ev3_dc.yu-h)
 *  \brief  EV3 DC Motors.
 *  \author  Vitaly Kravtsov (in4lio@gmail.com)
 *  \copyright  See the LICENSE file.
 */

#ifndef EV3_DC_H
#define EV3_DC_H

#ifdef  EV3_DC_IMPLEMENT
#define EV3_DC_EXT
#define EV3_DC_EXT_INIT( dec, init ) \
	dec = init
#define EV3_DC_EXT_C
#define EV3_DC_EXT_C_INIT( dec, init ) \
	dec = init
#define EV3_DC_INL
#else
#define EV3_DC_EXT extern
#define EV3_DC_EXT_INIT( dec, init ) \
	extern dec
#ifdef __cplusplus
#define EV3_DC_C "C"
#else
#define EV3_DC_C
#endif
#define EV3_DC_EXT_C extern EV3_DC_C
#define EV3_DC_EXT_C_INIT( dec, init ) \
	extern EV3_DC_C dec
#if __GNUC__ && !__GNUC_STDC_INLINE__
#define EV3_DC_INL extern inline
#else
#define EV3_DC_INL inline
#endif
#endif

#ifndef COMMA
#define COMMA ,
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/**
 *  \defgroup ev3_dc DC motors
 *  \brief Access to EV3 DC motors.
 *  \see http://www.ev3dev.org/docs/motors/
 *  \see http://www.ev3dev.org/docs/drivers/dc-motor-class/
 *  \{
 */

#define DC_DIR  "/sys/class/dc-motor"  /**< Directory of DC motors. */

/**
 *  \brief Structure of a DC motor descriptor.
 */
typedef struct {
	INX_T type_inx;  /**< DC motor type. */
	uint8_t port;  /**< DC motor EV3 port. */
	uint8_t extport;  /**< DC motor extended port. */

} EV3_DC;

#define DC_DESC__LIMIT_  DESC_LIMIT  /**< Limit of DC motor descriptors. */

#define DC__NONE_  DC_DESC__LIMIT_  /**< DC motor is not found. */

/**
 *  \brief Vector of DC motor descriptors (filled by \ref ev3_dc_init).
 */
EV3_DC_EXT_C EV3_DC ev3_dc[ DC_DESC__LIMIT_ ];

/**
 *  \brief Identifiers of DC motor types.
 */
enum {
	DC_TYPE__NONE_ = 0,  /* XXX: memset( 0 ) is used */

	RCX_MOTOR,

	DC_TYPE__COUNT_,  /**< Count of DC motor types. */
	DC_TYPE__UNKNOWN_ = DC_TYPE__COUNT_
};

/**
 *  \brief Common identifiers of DC motor "command" attribute.
 */
enum {
	DC_COMMAND__NONE_ = 0,

	DC_RUN_FOREVER,
	DC_RUN_TIMED,
	DC_RUN_DIRECT,
	DC_STOP,

	DC_COMMAND__COUNT_,  /**< Count of DC motor "command" attribute. */
	DC_COMMAND__UNKNOWN_ = DC_COMMAND__COUNT_
};

/**
 *  \brief Common identifiers of DC motor "polarity" attribute.
 */
enum {
	DC_POLARITY__NONE_ = 0,

	DC_NORMAL,
	DC_INVERSED,

	DC_POLARITY__COUNT_,  /**< Count of DC motor "polarity" attribute. */
	DC_POLARITY__UNKNOWN_ = DC_POLARITY__COUNT_
};

/**
 *  \brief Common identifiers of DC motor "stop_action" attribute.
 */
enum {
	DC_STOP_ACTION__NONE_ = 0,

	DC_COAST,
	DC_BRAKE,

	DC_STOP_ACTION__COUNT_,  /**< Count of DC motor "stop_action" attribute. */
	DC_STOP_ACTION__UNKNOWN_ = DC_STOP_ACTION__COUNT_
};

/**
 *  \brief Common identifiers of DC motor "state" attribute.
 */
enum {
	DC_STATE__NONE_ = 0,

	DC_RUNNING = 0x1L,
	DC_RAMPING = 0x2L,

};

/**
 *  \brief Read "address" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param[out] buf Buffer for result.
 *  \param sz Buffer size.
 *  \return Count of read bytes.
 */
EV3_DC_EXT_C size_t get_dc_address( uint8_t sn, char *buf, size_t sz );

/**
 *  \brief Write "command" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t set_dc_command( uint8_t sn, char *value );

/**
 *  \brief Write "command" attribute of several DC motors.
 *  \param sn Vector of sequence numbers ending with DESC_LIMIT.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t multi_set_dc_command( uint8_t *sn, char *value );

/**
 *  \brief Read "commands" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param[out] buf Buffer for result.
 *  \param sz Buffer size.
 *  \return Count of read bytes.
 */
EV3_DC_EXT_C size_t get_dc_commands( uint8_t sn, char *buf, size_t sz );

/**
 *  \brief Read "driver_name" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param[out] buf Buffer for result.
 *  \param sz Buffer size.
 *  \return Count of read bytes.
 */
EV3_DC_EXT_C size_t get_dc_driver_name( uint8_t sn, char *buf, size_t sz );

/**
 *  \brief Read "duty_cycle" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param[out] buf Buffer for result.
		 
 *  \return Count of read bytes.
 */
EV3_DC_EXT_C size_t get_dc_duty_cycle( uint8_t sn, int *buf );

/**
 *  \brief Read "duty_cycle_sp" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param[out] buf Buffer for result.
		 
 *  \return Count of read bytes.
 */
EV3_DC_EXT_C size_t get_dc_duty_cycle_sp( uint8_t sn, int *buf );

/**
 *  \brief Write "duty_cycle_sp" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t set_dc_duty_cycle_sp( uint8_t sn, int value );

/**
 *  \brief Write "duty_cycle_sp" attribute of several DC motors.
 *  \param sn Vector of sequence numbers ending with DESC_LIMIT.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t multi_set_dc_duty_cycle_sp( uint8_t *sn, int value );

/**
 *  \brief Read "polarity" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param[out] buf Buffer for result.
 *  \param sz Buffer size.
 *  \return Count of read bytes.
 */
EV3_DC_EXT_C size_t get_dc_polarity( uint8_t sn, char *buf, size_t sz );

/**
 *  \brief Write "polarity" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t set_dc_polarity( uint8_t sn, char *value );

/**
 *  \brief Write "polarity" attribute of several DC motors.
 *  \param sn Vector of sequence numbers ending with DESC_LIMIT.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t multi_set_dc_polarity( uint8_t *sn, char *value );

/**
 *  \brief Read "state" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param[out] buf Buffer for result.
 *  \param sz Buffer size.
 *  \return Count of read bytes.
 */
EV3_DC_EXT_C size_t get_dc_state( uint8_t sn, char *buf, size_t sz );

/**
 *  \brief Write "stop_action" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t set_dc_stop_action( uint8_t sn, char *value );

/**
 *  \brief Write "stop_action" attribute of several DC motors.
 *  \param sn Vector of sequence numbers ending with DESC_LIMIT.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t multi_set_dc_stop_action( uint8_t *sn, char *value );

/**
 *  \brief Read "stop_actions" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param[out] buf Buffer for result.
 *  \param sz Buffer size.
 *  \return Count of read bytes.
 */
EV3_DC_EXT_C size_t get_dc_stop_actions( uint8_t sn, char *buf, size_t sz );

/**
 *  \brief Read "ramp_down_sp" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param[out] buf Buffer for result.
		 
 *  \return Count of read bytes.
 */
EV3_DC_EXT_C size_t get_dc_ramp_down_sp( uint8_t sn, int *buf );

/**
 *  \brief Write "ramp_down_sp" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t set_dc_ramp_down_sp( uint8_t sn, int value );

/**
 *  \brief Write "ramp_down_sp" attribute of several DC motors.
 *  \param sn Vector of sequence numbers ending with DESC_LIMIT.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t multi_set_dc_ramp_down_sp( uint8_t *sn, int value );

/**
 *  \brief Read "ramp_up_sp" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param[out] buf Buffer for result.
		 
 *  \return Count of read bytes.
 */
EV3_DC_EXT_C size_t get_dc_ramp_up_sp( uint8_t sn, int *buf );

/**
 *  \brief Write "ramp_up_sp" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t set_dc_ramp_up_sp( uint8_t sn, int value );

/**
 *  \brief Write "ramp_up_sp" attribute of several DC motors.
 *  \param sn Vector of sequence numbers ending with DESC_LIMIT.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t multi_set_dc_ramp_up_sp( uint8_t *sn, int value );

/**
 *  \brief Read "time_sp" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param[out] buf Buffer for result.
		 
 *  \return Count of read bytes.
 */
EV3_DC_EXT_C size_t get_dc_time_sp( uint8_t sn, int *buf );

/**
 *  \brief Write "time_sp" attribute of the DC motor.
 *  \param sn Sequence number.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t set_dc_time_sp( uint8_t sn, int value );

/**
 *  \brief Write "time_sp" attribute of several DC motors.
 *  \param sn Vector of sequence numbers ending with DESC_LIMIT.
 *  \param value Attribute value.
		 
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t multi_set_dc_time_sp( uint8_t *sn, int value );

/**
 *  \brief Get name of the specified DC motor type.
 *  \param type_inx Index of the DC motor type.
 *  \return Requested value.
 */
EV3_DC_EXT_C const char *ev3_dc_type( INX_T type_inx );

/**
 *  \brief Read "driver_name" attribute and get index of the DC motor type.
 *  \param sn Sequence number.
 *  \return Requested value.
 */
EV3_DC_EXT_C INX_T get_dc_type_inx( uint8_t sn );

/**
 *  \brief Read DC motor attributes that are required for filling the descriptor.
 *  \param sn Sequence number.
 *  \param desc Buffer for the descriptor.
 *  \return Count of read bytes.
 */
EV3_DC_EXT_C size_t get_dc_desc( uint8_t sn, EV3_DC *desc );

/**
 *  \brief Get descriptor of the DC motor.
 *  \param sn Sequence number.
 *  \return Pointer to the DC motor descriptor.
 */
EV3_DC_EXT_C EV3_DC *ev3_dc_desc( uint8_t sn );

/**
 *  \brief Get type from the DC motor descriptor.
 *  \param sn Sequence number.
 *  \return Requested value.
 */
EV3_DC_EXT_C INX_T ev3_dc_desc_type_inx( uint8_t sn );

/**
 *  \brief Get EV3 port from the DC motor descriptor.
 *  \param sn Sequence number.
 *  \return Requested value.
 */
EV3_DC_EXT_C uint8_t ev3_dc_desc_port( uint8_t sn );

/**
 *  \brief Get extended port from the DC motor descriptor.
 *  \param sn Sequence number.
 *  \return Requested value.
 */
EV3_DC_EXT_C uint8_t ev3_dc_desc_extport( uint8_t sn );

/**
 *  \brief Assemble EV3 port name from the DC motor descriptor.
 *  \param sn Sequence number.
 *  \param[out] buf Buffer for result.
 *  \return Requested value.
 */
EV3_DC_EXT_C char *ev3_dc_port_name( uint8_t sn, char *buf );

/**
 *  \brief Search of a sequence number of the specified DC motor type.
 *  \param type_inx DC motor type.
 *  \param[out] sn Buffer for the sequence number.
 *  \param from Search initial value.
 *  \return Flag - the DC motor is found.
 */
EV3_DC_EXT_C bool ev3_search_dc( INX_T type_inx, uint8_t *sn, uint8_t from );

/**
 *  \brief Search of a sequence number the DC motor by plug-in attributes.
 *  \param port EV3 port.
 *  \param extport Extended port.
 *  \param[out] sn Buffer for the sequence number.
 *  \param from Search initial value.
 *  \return Flag - the DC motor is found.
 */
EV3_DC_EXT_C bool ev3_search_dc_plugged_in( uint8_t port, uint8_t extport, uint8_t *sn, uint8_t from );

/**
 *  \brief Get name of the specified DC motor command.
 *  \param command_inx Index of the DC motor command.
 *  \return Requested value.
 */
EV3_DC_EXT_C const char *ev3_dc_command( INX_T command_inx );

/**
 *  \brief Write "command" attribute of the DC motor by the index.
 *  \param sn Sequence number.
 *  \param command_inx Index of the DC motor command.
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t set_dc_command_inx( uint8_t sn, INX_T command_inx );

/**
 *  \brief Write "command" attribute of several DC motors by the index.
 *  \param sn Vector of sequence numbers ending with DESC_LIMIT.
 *  \param command_inx Index of the DC motor command.
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t multi_set_dc_command_inx( uint8_t *sn, INX_T command_inx );

/**
 *  \brief Get name of the specified DC motor polarity.
 *  \param polarity_inx Index of the DC motor polarity.
 *  \return Requested value.
 */
EV3_DC_EXT_C const char *ev3_dc_polarity( INX_T polarity_inx );

/**
 *  \brief Read "polarity" attribute of the DC motor and get the index.
 *  \param sn Sequence number.
 *  \return Requested value.
 */
EV3_DC_EXT_C INX_T get_dc_polarity_inx( uint8_t sn );

/**
 *  \brief Write "polarity" attribute of the DC motor by the index.
 *  \param sn Sequence number.
 *  \param polarity_inx Index of the DC motor polarity.
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t set_dc_polarity_inx( uint8_t sn, INX_T polarity_inx );

/**
 *  \brief Write "polarity" attribute of several DC motors by the index.
 *  \param sn Vector of sequence numbers ending with DESC_LIMIT.
 *  \param polarity_inx Index of the DC motor polarity.
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t multi_set_dc_polarity_inx( uint8_t *sn, INX_T polarity_inx );

/**
 *  \brief Get name of the specified DC motor stop_action.
 *  \param stop_action_inx Index of the DC motor stop_action.
 *  \return Requested value.
 */
EV3_DC_EXT_C const char *ev3_dc_stop_action( INX_T stop_action_inx );

/**
 *  \brief Write "stop_action" attribute of the DC motor by the index.
 *  \param sn Sequence number.
 *  \param stop_action_inx Index of the DC motor stop_action.
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t set_dc_stop_action_inx( uint8_t sn, INX_T stop_action_inx );

/**
 *  \brief Write "stop_action" attribute of several DC motors by the index.
 *  \param sn Vector of sequence numbers ending with DESC_LIMIT.
 *  \param stop_action_inx Index of the DC motor stop_action.
 *  \return Count of written bytes.
 */
EV3_DC_EXT_C size_t multi_set_dc_stop_action_inx( uint8_t *sn, INX_T stop_action_inx );

/**
 *  \brief Read "state" attribute of the DC motor and get the flags.
 *  \param sn Sequence number.
 *  \param flags Buffer for the flags.
 *  \return Count of read bytes.
 */
EV3_DC_EXT_C size_t get_dc_state_flags( uint8_t sn, FLAGS_T *flags );

/**
 *  \brief Detect connected DC motors.
 *  \return Number of found DC motors or -1 in case of an error.
 */
EV3_DC_EXT_C int ev3_dc_init( void );

/** \} */

#undef EV3_DC_EXT
#undef EV3_DC_EXT_INIT
#undef EV3_DC_EXT_C
#undef EV3_DC_EXT_C_INIT
#undef EV3_DC_INL
#undef EV3_DC_C
#endif

