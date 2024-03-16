/** \mainpage Driver Overview
  *
  * \section section_drv_info Driver Information
  * This Sensor Controller Interface driver has been generated by the Texas Instruments Sensor Controller
  * Studio tool:
  * - <b>Project name</b>:     SC Example
  * - <b>Project file</b>:     E:/SC/SC_Example.scp
  * - <b>Code prefix</b>:      -
  * - <b>Operating system</b>: TI Driver Porting Layer
  * - <b>Tool version</b>:     2.9.0.208
  * - <b>Tool patches</b>:     1, 2, 3 and 4
  * - <b>Target chip</b>:      CC1352P1F3, package QFN48 7x7 RGZ, revision E (2.1) or F (3.0)
  * - <b>Created</b>:          2024-03-15 16:53:53.014
  * - <b>Computer</b>:         BENNI-ASUS
  * - <b>User</b>:             Bened
  *
  * No user-provided resource definitions were used to generate this driver.
  *
  * No user-provided procedure definitions were used to generate this driver.
  *
  * Do not edit the generated source code files other than temporarily for debug purposes. Any
  * modifications will be overwritten by the Sensor Controller Studio when generating new output.
  *
  * \section section_drv_modules Driver Modules
  * The driver is divided into three modules:
  * - \ref module_scif_generic_interface, providing the API for:
  *     - Initializing and uninitializing the driver
  *     - Task control (for starting, stopping and executing Sensor Controller tasks)
  *     - Task data exchange (for producing input data to and consume output data from Sensor Controller
  *       tasks)
  * - \ref module_scif_driver_setup, containing:
  *     - The AUX RAM image (Sensor Controller code and data)
  *     - I/O mapping information
  *     - Task data structure information
  *     - Driver setup data, to be used in the driver initialization
  *     - Project-specific functionality
  * - \ref module_scif_osal, for flexible OS support:
  *     - Interfaces with the selected operating system
  *
  * It is possible to use output from multiple Sensor Controller Studio projects in one application. Only
  * one driver setup may be active at a time, but it is possible to switch between these setups. When
  * using this option, there is one instance of the \ref module_scif_generic_interface and
  * \ref module_scif_osal modules, and multiple instances of the \ref module_scif_driver_setup module.
  * This requires that:
  * - The outputs must be generated using the same version of Sensor Controller Studio
  * - The outputs must use the same operating system
  * - The outputs must use different source code prefixes (inserted into all globals of the
  *   \ref module_scif_driver_setup)
  *
  *
  * \section section_project_info Project Description
  * A basic example for the sensor controller
  *
  *
  * \subsection section_io_mapping I/O Mapping
  * Task I/O functions are mapped to the following pins:
  * - Button task:
  *     - <b>I: Button 1</b>: DIO15
  *     - <b>I: Button 2</b>: DIO14
  *
  *
  * \section section_task_info Task Description(s)
  * This driver supports the following task(s):
  *
  *
  * \subsection section_task_desc_button_task Button task
  * To wake the main CPU, push button 1 and in the next 5 seconds push button 2 at leats 5 times.
  *
  */




/** \addtogroup module_scif_driver_setup Driver Setup
  *
  * \section section_driver_setup_overview Overview
  *
  * This driver setup instance has been generated for:
  * - <b>Project name</b>:     SC Example
  * - <b>Code prefix</b>:      -
  *
  * The driver setup module contains the generated output from the Sensor Controller Studio project:
  * - Location of task control and scheduling data structures in AUX RAM
  * - The AUX RAM image, and the size the image
  * - Task data structure information (location, size and buffer count)
  * - I/O pin mapping translation table
  * - Task resource initialization and uninitialization functions
  * - Hooks for run-time logging
  *
  * @{
  */
#ifndef SCIF_H
#define SCIF_H

#include <stdint.h>
#include <stdbool.h>
#include "scif_framework.h"
#include "scif_osal_tidpl.h"


/// Target chip name
#define SCIF_TARGET_CHIP_NAME_CC1352P1F3
/// Target chip package
#define SCIF_TARGET_CHIP_PACKAGE_QFN48_7X7_RGZ

/// Number of tasks implemented by this driver
#define SCIF_TASK_COUNT 1

/// Button task: Task ID
#define SCIF_BUTTON_TASK_TASK_ID 0


/// Button task I/O mapping: Button 1
#define SCIF_BUTTON_TASK_DIO_I_BTN1 15
/// Button task I/O mapping: Button 2
#define SCIF_BUTTON_TASK_DIO_I_BTN2 14


// All shared data structures in AUX RAM need to be packed
#pragma pack(push, 2)


/// Button task: Task output data structure
typedef struct {
    uint16_t count; ///< Count of button pushes
} SCIF_BUTTON_TASK_OUTPUT_T;


/// Button task: Task state structure
typedef struct {
    uint16_t button1pressed; ///< Button 1 has been pressed and the interrupt executed
} SCIF_BUTTON_TASK_STATE_T;


/// Sensor Controller task data (configuration, input buffer(s), output buffer(s) and internal state)
typedef struct {
    struct {
        SCIF_BUTTON_TASK_OUTPUT_T output;
        SCIF_BUTTON_TASK_STATE_T state;
    } buttonTask;
} SCIF_TASK_DATA_T;

/// Sensor Controller task generic control (located in AUX RAM)
#define scifTaskData    (*((volatile SCIF_TASK_DATA_T*) 0x400E015C))


// Initialized internal driver data, to be used in the call to \ref scifInit()
extern const SCIF_DATA_T scifDriverSetup;


// Restore previous struct packing setting
#pragma pack(pop)


// AUX I/O re-initialization functions
void scifReinitTaskIo(uint32_t bvTaskIds);


// No task-specific API available


#endif
//@}


// Generated by BENNI-ASUS at 2024-03-15 16:53:53.014
