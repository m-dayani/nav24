//
// Created by root on 12/24/23.
//

#ifndef NAV24_PARAMETERBLUEPRINT_H
#define NAV24_PARAMETERBLUEPRINT_H

namespace NAV24 {

#define PARAM_DS "Process/DS"
#define PARAM_CAM "Input/Camera"
#define PARAM_REL "Input/Relations"
#define PARAM_OP "Process/OP"
#define PARAM_OUT "Output"

#define PKEY_NAME "name"

#define PKEY_INTERFACE "interface"
#define PKEY_IF_TYPE "type"
#define PKEY_IF_TARGET "target"
#define PKEY_IF_PORT "port"

#define PKEY_IMG_SIZE "resolution"
#define PKEY_CAM_FPS "fps"
#define PKEY_CAM_CALIB "calib"

#define PKEY_IMG_PATHS "ImagePathParams"
#define PKEY_SEQ_BASE "seqBase"
#define PKEY_IMG_BASE "imageBase"
#define PKEY_IMG_FILE "imageFile"
#define PKEY_POSE_FILE "gt"
#define PKEY_TS_FACT "tsFactor"
#define PKEY_POS_FIRST "gtPosFirst"
#define PKEY_QW_FIRST "gtQwFirst"

}

#endif //NAV24_PARAMETERBLUEPRINT_H
