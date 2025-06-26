#include "chassis_task.h"
/*
 * ƽ�������㼰��ѭ��
 */
balance_chassis_t balance_chassis;
static chassis_move_t chassis_move = {0};
int lqr_cal_flag;
int code_start = 0;
float sin_var = 0;
float value = -1;
void chassis_task(void const *pvParameters)
{
    Debug_RegisterVar(&code_start, "code_start", DVar_Int8);
    Debug_RegisterVar(&sin_var, "sin_var", DVar_Float);
    vTaskDelay(2000);
    TickType_t lasttick = xTaskGetTickCount();
    stop_joint_motor();
    joint_motor_enable();
    leg_init();
    leg_goto_zero_position();
//    joint_motor_save_pos();
		float mgggg;
    unsigned int count = 0;
    fly_flag = 0;
		while (1)//�˶���ѭ��
    {
        if (switch_is_up(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL]))
        {
            code_start = 1;
        }
        else if (switch_is_mid(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL]))
        {
            code_start = 2;
        }
        else if (switch_is_down(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL]))
        {
            code_start = 3;
        }
				
        if (error_flag == 1)
        {
        code_start = 3;
        }
				
				if (code_start == 1)
        {
            value = -1; 
					  filter_cal();
            VMC_calculate();     // ���µ�ǰ����״̬��
            leg_pushforce_cal(); // �Ȳ���ֱ������������
						leg_harmo_pid_cal(motion_resolve[0].phi0, motion_resolve[1].phi0);
            lqr_cal_flag = matrix_calculate();                                        // lqr����
            leg_control_cal();                                                        // �Ȳ������ۺϼ���
            joint_torque_calculate();                                                 // �ۺ��ſɱȼ������չؽڵ���������
            leg_speed_cal();
            if (mgggg <= 500)
            {
								mgggg++;
								DM8009_ctrl_motor(&CHASSIS_CAN, 1, 0, 0, 0, 0, -0.1);
								vTaskDelay(2);
								DM8009_ctrl_motor(&CHASSIS_CAN, 2, 0, 0, 0, 0, +0.1);
								vTaskDelay(2);
								DM8009_ctrl_motor(&CHASSIS_CAN, 3, 0, 0, 0, 0, +0.2);
								vTaskDelay(2);
								DM8009_ctrl_motor(&CHASSIS_CAN, 4, 0, 0, 0, 0, -0.2);
								vTaskDelay(2);
            }
            else
            {

								if (fly_flag == 0)//�ж�ģʽȫ��ʹ��
								{	
                torque_close_control(&CHASSIS_CAN, 1, -leg_control.driving_iq[0]/1);
                vTaskDelay(1);
							  torque_close_control(&CHASSIS_CAN, 2, +leg_control.driving_iq[1]/1); // ���ҷ����Ƿ���
								DM8009_ctrl_motor(&CHASSIS_CAN, 1, 0, 0, 0, 1, -motion_resolve[RIGHT].torque[BACK]);
								DM8009_ctrl_motor(&CHASSIS_CAN, 2, 0, 0, 0, 1, -motion_resolve[RIGHT].torque[FRONT]);
								DM8009_ctrl_motor(&CHASSIS_CAN, 3, 0, 0, 0, 1, -motion_resolve[LEFT].torque[FRONT]);
								DM8009_ctrl_motor(&CHASSIS_CAN, 4, 0, 0, 0, 1, -motion_resolve[LEFT].torque[BACK]);
								}
								else//���ģʽ����ͣת
							  {
								 	torque_close_control(&CHASSIS_CAN, 1, -0.1);
                  vTaskDelay(1);
							 	  torque_close_control(&CHASSIS_CAN, 2, 0.1); // ���ҷ����Ƿ���
   					      DM8009_ctrl_motor(&CHASSIS_CAN, 1, 0, 0, 0, 1, -motion_resolve[RIGHT].torque[BACK]);
								  DM8009_ctrl_motor(&CHASSIS_CAN, 2, 0, 0, 0, 1, -motion_resolve[RIGHT].torque[FRONT]);
					        DM8009_ctrl_motor(&CHASSIS_CAN, 3, 0, 0, 0, 1, -motion_resolve[LEFT].torque[FRONT]);
								  DM8009_ctrl_motor(&CHASSIS_CAN, 4, 0, 0, 0, 1, -motion_resolve[LEFT].torque[BACK]);
								}
            }
        }
				
        if (code_start == 2)//����Ԥ��ģʽ����λ�ؽڵ���������Ƴ�����
        {
					  value = -1;
            filter_cal();
            VMC_calculate();     // ���µ�ǰ����״̬��
            leg_pushforce_cal(); // �Ȳ���ֱ������������
						leg_harmo_pid_cal(motion_resolve[0].phi0, motion_resolve[1].phi0);
            lqr_cal_flag = matrix_calculate();                                        // lqr����
            leg_control_cal();                                                        // �Ȳ������ۺϼ���
            joint_torque_calculate();                                                 // �ۺ��ſɱȼ������չؽڵ���������
            leg_speed_cal();
            if (mgggg <= 500)
            {
								mgggg++;
								DM8009_ctrl_motor(&CHASSIS_CAN, 1, 0, 0, 0, 0, -0.1);
								vTaskDelay(2);
								DM8009_ctrl_motor(&CHASSIS_CAN, 2, 0, 0, 0, 0, +0.1);
								vTaskDelay(2);
								DM8009_ctrl_motor(&CHASSIS_CAN, 3, 0, 0, 0, 0, +0.2);
								vTaskDelay(2);
								DM8009_ctrl_motor(&CHASSIS_CAN, 4, 0, 0, 0, 0, -0.2);
								vTaskDelay(2);
            }
            else
            {

							fly_flag = 0;
						  DM8009_ctrl_motor(&CHASSIS_CAN, 1, -0.28, 0, 68, 2, 0);
						  vTaskDelay(2);
						  DM8009_ctrl_motor(&CHASSIS_CAN, 2, 0.28, 0, 68, 2, 0);
						  vTaskDelay(2);
						  DM8009_ctrl_motor(&CHASSIS_CAN, 3, 0.28, 0, 68, 2, 0);
						  vTaskDelay(2);
							DM8009_ctrl_motor(&CHASSIS_CAN, 4, -0.28, 0, 68, 2, 0);
							torque_close_control(&CHASSIS_CAN, 1, -leg_control.driving_iq[0]/1);
              vTaskDelay(1);
			  	    torque_close_control(&CHASSIS_CAN, 2, +leg_control.driving_iq[1]/1); // ���ҷ����Ƿ���
							 
            }
        }
				if(code_start == 3)//��������ģʽ
				{
					  filter_cal();
            VMC_calculate();     // ���µ�ǰ����״̬��
            leg_pushforce_cal(); // �Ȳ���ֱ������������
						leg_harmo_pid_cal(motion_resolve[0].phi0, motion_resolve[1].phi0);
            lqr_cal_flag = matrix_calculate();                                        // lqr����
            leg_control_cal();                                                        // �Ȳ������ۺϼ���
            joint_torque_calculate();                                                 // �ۺ��ſɱȼ������չؽڵ���������
            leg_speed_cal();
            DM8009_ctrl_motor(&CHASSIS_CAN, 1, 0, 0, 0, 2.5, value);
            vTaskDelay(2);
            DM8009_ctrl_motor(&CHASSIS_CAN, 2, 0, 0, 0, 2.5, -value);
            vTaskDelay(2);
            DM8009_ctrl_motor(&CHASSIS_CAN, 3, 0, 0, 0, 2.5, -value);
            vTaskDelay(2);
            DM8009_ctrl_motor(&CHASSIS_CAN, 4, 0, 0, 0, 2.5, value);
            vTaskDelay(2);

            torque_close_control(&CHASSIS_CAN, 1, -0.1);
            torque_close_control(&CHASSIS_CAN, 2, 0.1); // ���ҷ����Ƿ���

    // ���� value ����
            if (value < 1) 
			      {
            value += 0.005;
				    }
				vTaskDelayUntil(&lasttick, CHASSIS_CONTROL_TIME_MS);//���ڿ���
			   }
    }
}
const chassis_move_t *get_chassis_move_point(void)
{
    return &chassis_move;
}
