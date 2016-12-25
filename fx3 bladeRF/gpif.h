#ifndef GPIF_H_
#define GPIF_H_

/**
 * Supported GPIF configurations
 */
typedef enum {
    GPIF_CONFIG_DISABLED,   /**< GPIF is disabled */
    GPIF_CONFIG_RF_LINK,    /**< GPIF configured for streaming RF samples */
    GPIF_CONFIG_FPGA_LOAD,  /**< GPIF configured for FPGA loading */
} NuandGpifConfig;

/**
 * Reconfigure the GPIF (and PIB) for the specified configuration
 *
 * @param   config      Desired configuration
 *
 * @return CY_U3P_* return code
 */
CyU3PReturnStatus_t NuandConfigureGpif(NuandGpifConfig config);

#endif

