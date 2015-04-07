LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
        audio_a2dp_hw.c

LOCAL_C_INCLUDES+= . \
    $(LOCAL_PATH)/../utils/include \
    $(LOCAL_PATH)/../prf/include

# MStar Android Patch Begin
# support all audio path output through a2dp
LOCAL_C_INCLUDES+= \
        $(TOP)/external/tinyalsa/include \
        $(TOP)/system/media/audio_utils/include

LOCAL_SHARED_LIBRARIES := \
    libcutils \
    liblog \
    libbt-utils \
    libbt-prof \
    libtinyalsa \
    libaudioutils
# MStar Android Patch End

LOCAL_SHARED_LIBRARIES += \
        libpower

LOCAL_MODULE := audio.a2dp.rtk
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)
