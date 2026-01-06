/*
 * global.h
 *
 *  Created on: 2021��4��3��
 *      Author: zengz
 */

#ifndef INCLUDE_GLOBAL_H_
#define INCLUDE_GLOBAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// ����汾����
#define IS_RELEASE_VERSION 0 //0:���԰汾 1:�����汾 2:EMC������

// ����ģʽ����
#define TEST_MODE_DISABLED 0 // ���ò���ģʽ
#define TEST_MODE_ENABLED  1 // ���ò���ģʽ

// ���ݰ汾�������ò���ģʽ
#if IS_RELEASE_VERSION == 1
    // �����汾ǿ�ƽ��ò���ģʽ
    #undef TEST_MODE
    #define TEST_MODE TEST_MODE_DISABLED
    #if defined(DEBUG) || defined(_DEBUG)
        #warning "Release version detected: Test mode is forcibly disabled"
    #endif
#else
    // �Ƿ����汾�����ò���ģʽ
    #ifndef TEST_MODE
        #define TEST_MODE TEST_MODE_DISABLED // Ĭ�����ò���ģʽ
    #endif
#endif

// ��֤���յĲ���ģʽ����
#if IS_RELEASE_VERSION == 1 && TEST_MODE != TEST_MODE_DISABLED
    #error "Configuration error: TEST_MODE must be disabled in release version"
#endif
#define ZC_EAS_TYPE  01     //��ʱû�б���ȶ�һ��

#define EAS_CONTROLLER_TYPE  ((ZC_EAS_TYPE & 0xFF00) >> 8)
#define EAS_CONTROLLER_TYPE1 (ZC_EAS_TYPE & 0x00FF)
#define EAS_DeviceType		 1          //0����װ   //1����װ
#define EAS_DeviceSubtype1   3          //��ѹƽ̨
#define EAS_DeviceSubtype2   1          //����
#define EAS_HW_VER           102        //Ӳ���汾
#define EAS_SW_VER           102        //�����汾
#define EAS_SOFTVER_YEAR     25         //��
#define EAS_SOFTVER_MONTH    7
#define EAS_SOFTVER_DAY      12
#define EAS_MCU_TYPE         11          //MCU�ͺ� �����Ŀ ��о΢ 11 Ӣ����9877 12
#define EAS_SUPPLIER_CODE    3          //�ͻ����

// ����ΪBYDЭ��������ϱ��汾��
#define REPORT_SW_VER        10001      // ͨѶЭ���DID�ϱ������汾��
#define REPORT_SW_WEEK       19         // ͨѶЭ���DID�ϱ�������

#define REPORT_HW_VER        EAS_HW_VER // ͨѶЭ���DID�ϱ�Ӳ���汾��
#define REPORT_HW_YEAR       24         // ͨѶЭ���DID�ϱ�Ӳ����
#define REPORT_HW_MONTH      7          // ͨѶЭ���DID�ϱ�Ӳ����
#define REPORT_HW_WEEK       29         // ͨѶЭ���DID�ϱ�Ӳ����
#define REPORT_HW_DATE       7          // ͨѶЭ���DID�ϱ�Ӳ����

#define CONTROLLER_NUM  "BBL1"
#define CONTROLLER_NUM_SIZE   (sizeof(CONTROLLER_NUM) - 1)


#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_GLOBAL_H_ */
