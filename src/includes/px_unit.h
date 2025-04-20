/**
 * @file px_unit.h
 *
 * @brief Unit conversion macros and types for physics calculations.
 *
 * This file defines unit multipliers for different measurement systems
 * and provides functions to create force and impulse vectors with the
 * specified unit conversion.
 *
 * @note The default unit is meters, but this can be overridden by defining
 *       PDPHYZX_UNIT externally.
 */

#ifndef PDPHYZX_PX_UNIT_H
#define PDPHYZX_PX_UNIT_H

// Multipliers for each unit (relative to meters)
#define PDPHYZX_UNIT_MM 1000.0f
#define PDPHYZX_UNIT_CM 100.0f
#define PDPHYZX_UNIT_M 1.0f

// User sets PDPHYZX_UNIT externally (default is meters)
#ifndef PDPHYZX_UNIT
#define PDPHYZX_UNIT PDPHYZX_UNIT_M
#endif

#define PDPHYZX_UNIT_SQ (PDPHYZX_UNIT * PDPHYZX_UNIT)

#endif // PDPHYZX_PX_UNIT_H  
