#include "gnss_tdcp.h"
#include "gnss_sd.h"
#include "asg_rtcm_decode.h"
#include "gnss_math.h"
#include "rtk_seq_ekf.h"
#include "rtk_seq_float_ekf.h"
#include "gnss_nmea.h"
asg_epoch_data_tdcp g_cur_epoch_data = { {0} };
asg_epoch_data_tdcp g_base_tdcp_data = { {0} };
asensing_tdcp_pos_t g_base_pos = { 0 };
double g_sat2site_dist[MAXOBS] = { 0.0 };
double g_sat2site_unit[MAXOBS][3] = { {0.0} };
#define MAX_PARA_TDCP (3 + 1)
#define MAX_Q_ELE (MAX_PARA_TDCP) * (MAX_PARA_TDCP)
#define OMGE_DIV_CLIGHT (OMGE/CLIGHT)
extern double asensing_get_sat_wave_length(int sys, uint8_t code_type)
{
	double wave_length = 0.0;
	int freq_temp = 0, freq_tag = 0;
	char freq_str[3] = { '\0' };
	if (code_type > CODE_NONE && code_type <= MAXCODE)
	{
		const char* obs_code = code2obs(code_type, &freq_temp);
		freq_str[0] = obs_code[0];
		freq_tag = atoi(freq_str);
		if (SYS_GPS == sys)
		{
			if (1 == freq_tag)
			{
				wave_length = CLIGHT / FREQ1;
			}
			else if (2 == freq_tag)
			{
				wave_length = CLIGHT / FREQ2;
			}
			else if (5 == freq_tag)
			{
				wave_length = CLIGHT / FREQ5;
			}
		}
		else if (SYS_CMP == sys)
		{
			if (1 == freq_tag)
			{
				wave_length = CLIGHT / FREQ1_CMP;
			}
			else if (5 == freq_tag)
			{
				wave_length = CLIGHT / FREQ5;
			}
			else if (7 == freq_tag)
			{
				wave_length = CLIGHT / FREQ2_CMP;
			}
			else if (8 == freq_tag)
			{
				wave_length = CLIGHT / FREQ_B2_CMP;
			}
			else if (6 == freq_tag)
			{
				wave_length = CLIGHT / FREQ3_CMP;
			}
		}
		else if (SYS_GAL == sys)
		{
			if (1 == freq_tag)
			{
				wave_length = CLIGHT / FREQ1;
			}
			else if (5 == freq_tag)
			{
				wave_length = CLIGHT / FREQ5;
			}
			else if (7 == freq_tag)
			{
				wave_length = CLIGHT / FREQ7;
			}
			else if (8 == freq_tag)
			{
				wave_length = CLIGHT / FREQ8;
			}
			else if (6 == freq_tag)
			{
				wave_length = CLIGHT / FREQ6;
			}
		}
		else if (SYS_QZS == sys)
		{
			if (1 == freq_tag)
			{
				wave_length = CLIGHT / FREQ1;
			}
			else if (2 == freq_tag)
			{
				wave_length = CLIGHT / FREQ2;
			}
			else if (5 == freq_tag)
			{
				wave_length = CLIGHT / FREQ5;
			}
			else if (6 == freq_tag)
			{
				wave_length = CLIGHT / FREQ6;
			}
		}
		else if (SYS_SBS == sys)
		{
			if (1 == freq_tag)
			{
				wave_length = CLIGHT / FREQ1;
			}
			else if (5 == freq_tag)
			{
				wave_length = CLIGHT / FREQ5;
			}
		}
		else if (SYS_GLO == sys)
		{
			//add your code here
		}
	}
	return wave_length;
}
void updateCurEpochData(const asg_obs_t* cur_obs)
{
	int i = 0, j = 0, ori_channel_num = NFREQ + NEXOBS, sat_num = 0;
	int sys = 0, prn = 0;
	unsigned char* dest_channel_num = NULL;
	asg_obsd_t* sat_obs = NULL;
	asg_sat_tdcp* dest_sat_obs = NULL;
	double wave = 0.0;
	if ((cur_obs->n) <= 0)
	{
		return;
	}
	g_cur_epoch_data.obs_time = cur_obs->data[0].time;
	for (i = 0; i < 3; ++i)
	{
		g_cur_epoch_data.site_blh[i] = g_cur_epoch_data.site_v_enu[i] = 0.0;
	}
	for (i = 0; i < cur_obs->n; ++i)
	{
		sat_obs = (asg_obsd_t *)((cur_obs->data) + i);
		dest_sat_obs = g_cur_epoch_data.sat_data + sat_num;
		for (j = 0; j < 4; ++j)
		{
			dest_sat_obs->sat_pos_clk[j] = dest_sat_obs->sat_v[j] = 0.0;
		}
		dest_sat_obs->sat = sat_obs->sat;
		dest_channel_num = &(dest_sat_obs->chann_num);
		*dest_channel_num = 0;
		for (j = 0; j < ori_channel_num; ++j)
		{
			if (CODE_NONE == sat_obs->code[j])
			{
				continue;
			}
			if ((fabs(sat_obs->L[j]) < 1.0e-3 && fabs(sat_obs->D[j]) < 1.0e-3) || fabs(sat_obs->P[j]) < 1.0e-3)
			{
				continue;
			}
			dest_sat_obs->SNR[*dest_channel_num] = sat_obs->SNR[j];
			dest_sat_obs->LLI[*dest_channel_num] = sat_obs->LLI[j];
			dest_sat_obs->code[*dest_channel_num] = sat_obs->code[j];
			dest_sat_obs->L[*dest_channel_num] = sat_obs->L[j];
			dest_sat_obs->P[*dest_channel_num] = sat_obs->P[j];
			sys = satsys(sat_obs->sat, &prn);
			wave = asensing_get_sat_wave_length(sys, sat_obs->code[j]);
			dest_sat_obs->D[*dest_channel_num] = (float)(sat_obs->D[j]* wave * (-1.0));
			++(*dest_channel_num);
		}
		if (*dest_channel_num > 0)
		{
			++sat_num;
		}
	}
	g_cur_epoch_data.sat_num = sat_num;
	return;
}
void pushBaseTDCPdata(const asg_epoch_data_tdcp* base_tdcp_data, const asensing_tdcp_pos_t* tdcp_pos)
{
	if (base_tdcp_data && tdcp_pos && tdcp_pos->fix_quality)
	{
		memset(&g_base_tdcp_data, 0, sizeof(asg_epoch_data_tdcp));
		g_base_tdcp_data = *base_tdcp_data;
		g_base_pos = *tdcp_pos;
	}
	return;
}
double get_tot_time(asg_gtime_t* gtime_tor,int gnss_model, double pseudo_range)
{
	double tot = 0.0, tor = 0.0;
	tor= rtklib_time2gpst(*gtime_tor, NULL);
	if ((gnss_model == GPS_MODE) || (gnss_model == GAL_MODE)) 
	{
		tot = tor - pseudo_range / CLIGHT;
	}
	else if (gnss_model == BDS_MODE)
	{
		tot = tor - 14 - pseudo_range / CLIGHT;
	}
	else if (gnss_model == GLN_MODE) 
	{
		// need to add
	}
	if (gnss_model != GLN_MODE && tot < 0) 
	{
		tot += 604800;
	}
	return tot;
}
void fillup_rover_sat_info(asg_gtime_t* cur_time)
{
	int i = 0, j = 0, sys = 0, sys_model = 0, prn = 0, obs_valid = 0;
	asg_sat_tdcp* sat_data = NULL;
	sat_data_t* sp = NULL;
	double pseudo = 0.0, sat_clk = 0.0, tot = 0.0;
	ECEF svp = { 0 };
	double dopp_corr = 0.0;
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		sys = satsys(sat_data->sat, &prn);
		sys_model = GNSS_SYS2MODE(sys);
		sp = gnss_sd_get_sv_data(sys_model, prn, 0);
		if (NULL == sp)
		{
			continue;
		}
		obs_valid = 0;
		for (j = 0; j < sat_data->chann_num; ++j)
		{
			if (sat_data->P[j] > 1.0e-2)
			{
				pseudo = (sat_data->P[j]);
				obs_valid = 1;
				break;
			}
		}
		if (0 == obs_valid)
		{
			continue;
		}
		tot = get_tot_time(cur_time, sys_model, pseudo);
		if (tot <= 1.0e-6)
		{
			continue;
		}
		if (tot < 0.0)
		{
			if (sys_model == GLN_MODE)
			{
				tot += SECS_IN_DAY;
			}
			else
			{
				tot += (double)SECS_IN_WEEK;
			}
		}
		sat_clk = gnss_Sd_Clk(sys_model, prn, tot, &dopp_corr, 0);
		if (sys_model == GLN_MODE)
		{
			tot += sat_clk;
		}
		else
		{
			tot -= sat_clk;
		}
		sat_clk = gnss_Sd_Pos_e(sys_model, prn, tot, &svp, &dopp_corr, 0);
		if (sys_model == GLN_MODE)
		{
			sp->svt[0] = -sat_clk;
			sp->svt[1] = -dopp_corr;
		}
		else
		{
			sp->svt[0] = sat_clk;
			sp->svt[1] = dopp_corr;
		}
		for (j = 0; j < 3; ++j)
		{
			sat_data->sat_pos_clk[j] = svp.p[j];
			sat_data->sat_v[j] = svp.v[j];
		}
		sat_data->sat_pos_clk[3] = sp->svt[0];
		sat_data->sat_v[3] = sp->svt[1];
	}
	return;
}
void doppler_detect_slip(unsigned char* slip_tag,int index)
{
	int i = 0, j = 0;
	int sys = 0, prn = 0;
	double avg_doppler = 0.0, dist_var = 0.0, dist_var_carrier = 0.0;
	double wave1 = 0.0;
	asg_sat_tdcp* sat_data = NULL, * base_sat_data = NULL;
	const char* obs_code = NULL;
	int freq_temp = 0, freq_tag = 0;
	char freq_str[3] = { '\0' };
	double delta_t = asg_timediff(g_cur_epoch_data.obs_time, g_base_tdcp_data.obs_time);
	double diff[MAXOBS] = { 0.0 }, mean = 0.0;
	int n = 0;
	//uint8_t flag = 0;

	const char* base_obs_code = NULL;
	int base_freq_temp = 0, base_freq_tag = 0;
	char base_freq_str[3] = { '\0' };
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		sys = satsys(sat_data->sat, &prn);
		base_sat_data = g_base_tdcp_data.sat_data + j;
		
		wave1 = asensing_get_sat_wave_length(sys, sat_data->code[index]);
		//using doppler observations to detect cycle slip
		obs_code = code2obs(sat_data->code[index], &freq_temp);
		freq_str[0] = obs_code[0];
		freq_tag = atoi(freq_str);

		base_obs_code = code2obs(base_sat_data->code[index], &base_freq_temp);
		base_freq_str[0] = base_obs_code[0];
		base_freq_tag = atoi(base_freq_str);

		avg_doppler = 0.0;
		if (freq_tag == base_freq_tag)
		{
			if (fabs(sat_data->D[index]) > 0.0 && fabs(base_sat_data->D[index]) > 0.0)
			{
				avg_doppler = ((double)sat_data->D[index] + base_sat_data->D[index]) * 0.5;
			}
			else if (fabs(sat_data->D[index]) > 0.0)
			{
				avg_doppler = sat_data->D[index];
			}
			else if (fabs(base_sat_data->D[index]) > 0.0)
			{
				avg_doppler = base_sat_data->D[index];
			}
		}

		if (fabs(avg_doppler) > 1.0e-3 && fabs(base_sat_data->L[index]) > 0.0 && fabs(sat_data->L[index]) > 0.0)
		{
			dist_var = avg_doppler * delta_t / wave1;
			dist_var_carrier = (sat_data->L[index] - base_sat_data->L[index]);
			diff[n++] = dist_var_carrier - dist_var;
		}
		//end
	}
	(void)gnss_median_dbl(diff,n, &mean);
	n = 0;
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		if ((sat_data->LLI[0] & 0x1) == 0x1 && sat_data->SNR[0] < 42)
		{
			continue;
		}
		sys = satsys(sat_data->sat, &prn);
		base_sat_data = g_base_tdcp_data.sat_data + j;

		wave1 = asensing_get_sat_wave_length(sys, sat_data->code[index]);
		//using doppler observations to detect cycle slip
		obs_code = code2obs(sat_data->code[index], &freq_temp);
		freq_str[0] = obs_code[0];
		freq_tag = atoi(freq_str);

		base_obs_code = code2obs(base_sat_data->code[index], &base_freq_temp);
		base_freq_str[0] = base_obs_code[0];
		base_freq_tag = atoi(base_freq_str);

		avg_doppler = 0.0;
		if (freq_tag == base_freq_tag)
		{
			if (fabs(sat_data->D[index]) > 0.0 && fabs(base_sat_data->D[index]) > 0.0)
			{
				avg_doppler = ((double)sat_data->D[index] + base_sat_data->D[index]) * 0.5;
			}
			else if (fabs(sat_data->D[index]) > 0.0)
			{
				avg_doppler = sat_data->D[index];
			}
			else if (fabs(base_sat_data->D[index]) > 0.0)
			{
				avg_doppler = base_sat_data->D[index];
			}
		}

		if (fabs(avg_doppler) > 1.0e-3 && fabs(base_sat_data->L[index]) > 0.0 && fabs(sat_data->L[index]) > 0.0)
		{
			dist_var = avg_doppler * delta_t / wave1;
			dist_var_carrier = (sat_data->L[index] - base_sat_data->L[index]);
			diff[n] = dist_var_carrier - dist_var - mean;
			if (fabs(diff[n]) < 1.0)
			{
				slip_tag[i * (NFREQ + NEXOBS) +index] |= 2;
			}
			n++;
		}
		//end
	}
	return;
}
void doppler_detect_pseudo_range(unsigned char* slip_tag, int index)
{
	int i = 0, j = 0;
	//int sys = 0, prn = 0;
	double avg_doppler = 0.0, dist_var = 0.0, dist_var_carrier = 0.0;
	//double wave1 = 0.0;
	asg_sat_tdcp* sat_data = NULL, * base_sat_data = NULL;
	const char* obs_code = NULL;
	int freq_temp = 0, freq_tag = 0;
	char freq_str[3] = { '\0' };
	double delta_t = asg_timediff(g_cur_epoch_data.obs_time, g_base_tdcp_data.obs_time);
	double diff[MAXOBS] = { 0.0 }, mean = 0.0;
	int n = 0;
	//uint8_t flag = 0;

	const char* base_obs_code = NULL;
	int base_freq_temp = 0, base_freq_tag = 0;
	char base_freq_str[3] = { '\0' };
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		//sys = satsys(sat_data->sat, &prn);
		base_sat_data = g_base_tdcp_data.sat_data + j;

		//using doppler observations to detect gross error for pseudo-range observations
		obs_code = code2obs(sat_data->code[index], &freq_temp);
		freq_str[0] = obs_code[0];
		freq_tag = atoi(freq_str);

		base_obs_code = code2obs(base_sat_data->code[index], &base_freq_temp);
		base_freq_str[0] = base_obs_code[0];
		base_freq_tag = atoi(base_freq_str);

		avg_doppler = 0.0;
		if (freq_tag == base_freq_tag)
		{
			if (fabs(sat_data->D[index]) > 0.0 && fabs(base_sat_data->D[index]) > 0.0)
			{
				avg_doppler = ((double)sat_data->D[index] + base_sat_data->D[index]) * 0.5;
			}
			else if (fabs(sat_data->D[index]) > 0.0)
			{
				avg_doppler = sat_data->D[index];
			}
			else if (fabs(base_sat_data->D[index]) > 0.0)
			{
				avg_doppler = base_sat_data->D[index];
			}
		}

		if (fabs(avg_doppler) > 1.0e-3 && fabs(base_sat_data->P[index]) > 0.0 && fabs(sat_data->P[index]) > 0.0)
		{
			dist_var = avg_doppler * delta_t;
			dist_var_carrier = (sat_data->P[index] - base_sat_data->P[index]);
			diff[n++] = dist_var_carrier - dist_var;
		}
		//end
	}
	(void)gnss_median_dbl(diff, n, &mean);
	n = 0;
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		//sys = satsys(sat_data->sat, &prn);
		base_sat_data = g_base_tdcp_data.sat_data + j;

		//using doppler observations to detect cycle slip
		obs_code = code2obs(sat_data->code[index], &freq_temp);
		freq_str[0] = obs_code[0];
		freq_tag = atoi(freq_str);

		base_obs_code = code2obs(base_sat_data->code[index], &base_freq_temp);
		base_freq_str[0] = base_obs_code[0];
		base_freq_tag = atoi(base_freq_str);

		avg_doppler = 0.0;
		if (freq_tag == base_freq_tag)
		{
			if (fabs(sat_data->D[index]) > 0.0 && fabs(base_sat_data->D[index]) > 0.0)
			{
				avg_doppler = ((double)sat_data->D[index] + base_sat_data->D[index]) * 0.5;
			}
			else if (fabs(sat_data->D[index]) > 0.0)
			{
				avg_doppler = sat_data->D[index];
			}
			else if (fabs(base_sat_data->D[index]) > 0.0)
			{
				avg_doppler = base_sat_data->D[index];
			}
		}

		if (fabs(avg_doppler) > 1.0e-3 && fabs(base_sat_data->P[index]) > 0.0 && fabs(sat_data->P[index]) > 0.0)
		{
			dist_var = avg_doppler * delta_t;
			dist_var_carrier = (sat_data->P[index] - base_sat_data->P[index]);
			diff[n] = dist_var_carrier - dist_var - mean;
			if (fabs(diff[n]) < 10.0)
			{
				slip_tag[i * (NFREQ + NEXOBS) + index] |= 2;
			}
			n++;
		}
		//end
	}
	return;
}
void corse_detect_slip(unsigned char* slip_tag)
{
	int i = 0, j = 0, dual_freq_valid = 0, base_dual_freq_valid = 0;
	int sys = 0, prn = 0, index = 0;
	double cur_gf = 0.0, base_gf = 0.0, delta_gf = 0.0/*, avg_doppler = 0.0, dist_var = 0.0, dist_var_carrier = 0.0, dist_diff = 0.0*/;
	double wave1 = 0.0, wave2 = 0.0, base_wave1 = 0.0, base_wave2 = 0.0;
	asg_sat_tdcp* sat_data = NULL, *base_sat_data = NULL;
	//const char* obs_code = NULL;
	//int freq_temp = 0, freq_tag = 0;
	//char freq_str[3] = { '\0' };
	
	//const char* base_obs_code = NULL;
	//int base_freq_temp = 0, base_freq_tag = 0;
	//char base_freq_str[3] = { '\0' };
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		if ((sat_data->LLI[0] & 0x1) == 0x1 && sat_data->SNR[0] < 42)
		{
			continue;
		}
		sys = satsys(sat_data->sat, &prn);
		base_sat_data = g_base_tdcp_data.sat_data + j;
		dual_freq_valid = 0;
		if (fabs(sat_data->L[0]) > 0.0 && fabs(sat_data->L[1]) > 0.0)
		{
			dual_freq_valid = 1;
		}
		base_dual_freq_valid = 0;
		if (fabs(base_sat_data->L[0]) > 0.0 && fabs(base_sat_data->L[1]) > 0.0)
		{
			base_dual_freq_valid = 1;
		}

		if (1 == dual_freq_valid && 1 == base_dual_freq_valid)
		{
			wave1 = asensing_get_sat_wave_length(sys, sat_data->code[0]);
			wave2 = asensing_get_sat_wave_length(sys, sat_data->code[1]);
			base_wave1 = asensing_get_sat_wave_length(sys, base_sat_data->code[0]);
			base_wave2 = asensing_get_sat_wave_length(sys, base_sat_data->code[1]);
			if (wave1 > 0.0 && wave2 > 0.0 && base_wave1 > 0.0 && base_wave2 > 0.0)
			{
				cur_gf = wave1 * (sat_data->L[0]) - wave2 * (sat_data->L[1]);
				base_gf = base_wave1 * (base_sat_data->L[0]) - base_wave2 * (base_sat_data->L[1]);
				delta_gf = fabs(cur_gf - base_gf);
				if (delta_gf < 0.1)
				{
					index = i*(NFREQ+NEXOBS);
					slip_tag[index] |= 1;
					index = i * (NFREQ + NEXOBS) + 1;
					slip_tag[index] |= 1;
				}
			}
		}
	}
	for (i = 0; i < NFREQ + NEXOBS; ++i)
	{
		doppler_detect_slip(slip_tag, i);
	}
	return;
}
int getRcvClkDriftTDCP(unsigned char* slip_tag, double* rcv_clk_drift)
{
	int i = 0, j = 0, k = 0;
	int sys = 0, prn = 0, index = 0;
	double wave = 0.0;
	asg_sat_tdcp* sat_data = NULL, * base_sat_data = NULL;
	const char* obs_code = NULL;
	int freq_temp = 0, freq_tag = 0;
	char freq_str[3] = { '\0' };

	const char* base_obs_code = NULL;
	int base_freq_temp = 0, base_freq_tag = 0;
	char base_freq_str[3] = { '\0' };
	//double r = 0.0, e[3] = { 0.0 }, base_r = 0.0, base_e[3] = { 0.0 };
	double diff[MAXOBS] = { 0.0 }, mean = 0.0;
	uint8_t flag = FALSE;
	int n = 0;
	int sat_info_valid = 0;
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		sys = satsys(sat_data->sat, &prn);
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		base_sat_data = g_base_tdcp_data.sat_data + j;
		sat_info_valid = 1;
		for (k = 0; k < 3; ++k)
		{
			if (fabs(sat_data->sat_pos_clk[k]) < 1.0e-2 || fabs(base_sat_data->sat_pos_clk[k]) < 1.0e-2)
			{
				sat_info_valid = 0;
				break;
			}
		}
		if (0 == sat_info_valid)
		{
			continue;
		}
		for (k = 0; k < 1; ++k)
		{
			index = i*(NFREQ + NEXOBS) + k;
			obs_code = code2obs(sat_data->code[k], &freq_temp);
			freq_str[0] = obs_code[0];
			freq_tag = atoi(freq_str);

			base_obs_code = code2obs(base_sat_data->code[k], &base_freq_temp);
			base_freq_str[0] = base_obs_code[0];
			base_freq_tag = atoi(base_freq_str);

			if (freq_tag != base_freq_tag)
			{
				continue;
			}
			wave = asensing_get_sat_wave_length(sys, sat_data->code[k]);
			if (1 == slip_tag[index] || 2 == slip_tag[index] || 3 == slip_tag[index])
			{
				diff[n++] = (sat_data->L[k] - base_sat_data->L[k]) * wave - g_sat2site_dist[i];
			}
		}
	}
	flag = gnss_median_dbl(diff, n, &mean);
	if (TRUE == flag && NULL != rcv_clk_drift)
	{
		*rcv_clk_drift = mean;
	}
	return n;
}

int getRcvClkDriftTDPR(unsigned char* pseudo_gross_tag, double* rcv_clk_drift)
{
	int i = 0, j = 0, k = 0;
	int /*sys = 0, prn = 0, */index = 0;
	//double wave = 0.0;
	asg_sat_tdcp* sat_data = NULL, * base_sat_data = NULL;
	const char* obs_code = NULL;
	int freq_temp = 0, freq_tag = 0;
	char freq_str[3] = { '\0' };

	const char* base_obs_code = NULL;
	int base_freq_temp = 0, base_freq_tag = 0;
	char base_freq_str[3] = { '\0' };
	//double r = 0.0, e[3] = { 0.0 }, base_r = 0.0, base_e[3] = { 0.0 };
	double diff[MAXOBS] = { 0.0 }, mean = 0.0;
	uint8_t flag = FALSE;
	int n = 0;
	int sat_info_valid = 0;
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		//sys = satsys(sat_data->sat, &prn);
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		base_sat_data = g_base_tdcp_data.sat_data + j;
		sat_info_valid = 1;
		for (k = 0; k < 3; ++k)
		{
			if (fabs(sat_data->sat_pos_clk[k]) < 1.0e-2 || fabs(base_sat_data->sat_pos_clk[k]) < 1.0e-2)
			{
				sat_info_valid = 0;
				break;
			}
		}
		if (0 == sat_info_valid)
		{
			continue;
		}
		for (k = 0; k < 1; ++k)
		{
			index = i * (NFREQ + NEXOBS) + k;
			obs_code = code2obs(sat_data->code[k], &freq_temp);
			freq_str[0] = obs_code[0];
			freq_tag = atoi(freq_str);

			base_obs_code = code2obs(base_sat_data->code[k], &base_freq_temp);
			base_freq_str[0] = base_obs_code[0];
			base_freq_tag = atoi(base_freq_str);

			if (freq_tag != base_freq_tag)
			{
				continue;
			}
			if (0 != pseudo_gross_tag[index])
			{
				diff[n++] = (sat_data->P[k] - base_sat_data->P[k]) - g_sat2site_dist[i];
			}
		}
	}
	flag = gnss_median_dbl(diff, n, &mean);
	if (TRUE == flag && NULL != rcv_clk_drift)
	{
		*rcv_clk_drift = mean;
	}
	return n;
}

int getRcvClkDriftTDDR(unsigned char* pseudo_gross_tag, double* rcv_clk_drift)
{
	int i = 0, j = 0, k = 0;
	int /*sys = 0, prn = 0, */index = 0;
	//double wave = 0.0;
	asg_sat_tdcp* sat_data = NULL, * base_sat_data = NULL;
	const char* obs_code = NULL;
	int freq_temp = 0, freq_tag = 0;
	char freq_str[3] = { '\0' };

	const char* base_obs_code = NULL;
	int base_freq_temp = 0, base_freq_tag = 0;
	char base_freq_str[3] = { '\0' };
	//double r = 0.0, e[3] = { 0.0 }, base_r = 0.0, base_e[3] = { 0.0 };
	double diff[MAXOBS] = { 0.0 }, mean = 0.0, avg_doppler = 0.0;
	uint8_t flag = FALSE;
	int n = 0;
	int sat_info_valid = 0;
	double delta_t = asg_timediff(g_cur_epoch_data.obs_time, g_base_tdcp_data.obs_time);
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		//sys = satsys(sat_data->sat, &prn);
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		base_sat_data = g_base_tdcp_data.sat_data + j;
		sat_info_valid = 1;
		for (k = 0; k < 3; ++k)
		{
			if (fabs(sat_data->sat_pos_clk[k]) < 1.0e-2 || fabs(base_sat_data->sat_pos_clk[k]) < 1.0e-2)
			{
				sat_info_valid = 0;
				break;
			}
		}
		if (0 == sat_info_valid)
		{
			continue;
		}
		for (k = 0; k < 1; ++k)
		{
			index = i * (NFREQ + NEXOBS) + k;
			obs_code = code2obs(sat_data->code[k], &freq_temp);
			freq_str[0] = obs_code[0];
			freq_tag = atoi(freq_str);

			base_obs_code = code2obs(base_sat_data->code[k], &base_freq_temp);
			base_freq_str[0] = base_obs_code[0];
			base_freq_tag = atoi(base_freq_str);

			if (freq_tag != base_freq_tag)
			{
				continue;
			}
			if (0 != pseudo_gross_tag[index])
			{
				avg_doppler = 0.0;
				if (fabs(sat_data->D[k]) > 0.0 && fabs(base_sat_data->D[k]) > 0.0)
				{
					avg_doppler = ((double)sat_data->D[k] + base_sat_data->D[k]) * 0.5;
				}
				else if (fabs(sat_data->D[k]) > 0.0)
				{
					avg_doppler = sat_data->D[k];
				}
				else if (fabs(base_sat_data->D[k]) > 0.0)
				{
					avg_doppler = base_sat_data->D[k];
				}
				diff[n++] = avg_doppler*delta_t - g_sat2site_dist[i];
			}
		}
	}
	flag = gnss_median_dbl(diff, n, &mean);
	if (TRUE == flag && NULL != rcv_clk_drift)
	{
		*rcv_clk_drift = mean;
	}
	return n;
}

void sat_earth_rot_corr(const double* ori_sat_corr, double* rot_sat_corr, double tran_time)
{
	double omega_t = tran_time * OMGE;
	double sin_omega_t = sin(omega_t);
	double cos_omega_t = cos(omega_t);
	rot_sat_corr[0] = ori_sat_corr[0] * cos_omega_t + ori_sat_corr[1] * sin_omega_t;
	rot_sat_corr[1] = -ori_sat_corr[0] * sin_omega_t + ori_sat_corr[1] * cos_omega_t;
	rot_sat_corr[2] = ori_sat_corr[2];
	return;
}
float get_wp(double k0, double k1, double sigma0,double res)
{
	double wp = 1.0;
	double vDivideSigma = fabs(res / sigma0);
	double temp = 0.0;
	if (vDivideSigma <= k0)
	{
		wp = 1.0;
	}
	else if (vDivideSigma <= k1)
	{
		temp = (k1 - vDivideSigma) / (k1 - k0);
		wp = k0 / vDivideSigma * temp * temp;
	}
	else
	{
		wp = 1.0e-12;
	}
	return (float)wp;
}
double geodist_tdcp(const double* rs, const double* rr, double* e)
{
	double r = 0.0;
	int i = 0;

	if (dot(rs,rs, 3) < RE_WGS84_POW2)
	{
		return -1.0;
	}
	for (i = 0; i < 3; i++)
	{
		e[i] = rs[i] - rr[i];
	}
	r = asg_norm(e, 3);
	for (i = 0; i < 3; i++)
	{
		e[i] /= r;
	}
	return r;
}
double get_site2sat_dist(const asg_sat_tdcp* sat_data, const asg_sat_tdcp* base_sat_data, 
	const double xyz[3],double e[3])
{
	double r = 0.0, base_r = 0.0, base_e[3] = { 0.0 };
	double delta_dist = 0.0, earth_rot = 0.0;
	const double* sat_coor_pointer = sat_data->sat_pos_clk, * base_sat_coor_pointer = base_sat_data->sat_pos_clk;
	int k = 0;
	for (k = 0; k < 3; k++)
	{
		e[k] = sat_data->sat_pos_clk[k] - xyz[k];
	}
	r = asg_norm(e, 3);
	for (k = 0; k < 3; k++)
	{
		e[k] /= r;
	}
	for (k = 0; k < 3; k++)
	{
		base_e[k] = base_sat_data->sat_pos_clk[k] - xyz[k];
	}
	base_r = asg_norm(base_e, 3);
	earth_rot = ((sat_coor_pointer[0] - base_sat_coor_pointer[0]) * xyz[1] - (sat_coor_pointer[1] - base_sat_coor_pointer[1]) * xyz[0]) * OMGE_DIV_CLIGHT;
	delta_dist = r - base_r + earth_rot - (sat_data->sat_pos_clk[3] - base_sat_data->sat_pos_clk[3]) * CLIGHT;
	return delta_dist;
}
int compare_code_consist(int sys, int code_index,const asg_sat_tdcp* sat_data,
	const asg_sat_tdcp* base_sat_data, double* wave)
{
	int is_consist = 0;
	const char* obs_code = NULL;
	int freq_temp = 0, freq_tag = 0;
	char freq_str[3] = { '\0' };

	const char* base_obs_code = NULL;
	int base_freq_temp = 0, base_freq_tag = 0;
	char base_freq_str[3] = { '\0' };
	if (wave)
	{
		*wave = 0.0;
	}
	obs_code = code2obs(sat_data->code[code_index], &freq_temp);
	freq_str[0] = obs_code[0];
	freq_tag = atoi(freq_str);

	base_obs_code = code2obs(base_sat_data->code[code_index], &base_freq_temp);
	base_freq_str[0] = base_obs_code[0];
	base_freq_tag = atoi(base_freq_str);

	if (freq_tag == base_freq_tag)
	{
		is_consist = 1;
	}
	if (wave)
	{
		*wave = asensing_get_sat_wave_length(sys, sat_data->code[code_index]);
	}
	return is_consist;
}
int feedup_obs_filter(double rcv_clk_drift[GNSS_MAX_MODE], const double ori_xyz[3], unsigned char* slip_tag, double delta_x[3], double cur_xyz[3], float* wp,
	double* sigma0, int* res_less_005_sat, int* para_num)
{
	double Q[MAX_Q_ELE] = { 0.0 };
	double X[MAX_PARA_TDCP] = { 0.0 };
	double delta_para[MAX_PARA_TDCP] = { 0.0 };
	int i = 0, j = 0, k = 0, m = 0, enter_filter_num = 0;
	int sys = 0, prn = 0, index = 0, sys_model = 0;
	//double wave1 = 0.0, wave2 = 0.0, base_wave1 = 0.0, base_wave2 = 0.0;
	asg_sat_tdcp* sat_data = NULL, * base_sat_data = NULL;
	
	double e[3] = { 0.0 }, rad_30 = 30.0 * D2R;
	double wave = 0.0, delta_dist = 0.0, omc = 0.0, ele = 0.0, sin_ele = 0.0, weight = 1.0/*, res = 0.0*/;
	int sat_info_valid = 0, valid_obs_num = 0;
	int sat_obs_valid = 0, sat_valid_num = 0, sys_valid_obs[GNSS_MAX_MODE] = { 0 }, valid_sys_num = 0;
	int res_less_005_num = 0;
	*sigma0 = 0.0;
	*res_less_005_sat = 0;
	*para_num = 0;
	for (i = 0; i < 3; ++i)
	{
		Q[i * MAX_PARA_TDCP + i] = 1.0e6;
	}
	for (i = 3; i < MAX_PARA_TDCP; ++i)
	{
		Q[i * MAX_PARA_TDCP + i] = 1.0e4;
	}
	initH(MAX_PARA_TDCP);
	//carrier phase enter into kalman filter
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		sys = satsys(sat_data->sat, &prn);
		sys_model = GNSS_SYS2MODE(sys);
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		base_sat_data = g_base_tdcp_data.sat_data + j;
		sat_info_valid = 1;
		for (k = 0; k < 3; ++k)
		{
			if (fabs(sat_data->sat_pos_clk[k]) < 1.0e-2 || fabs(base_sat_data->sat_pos_clk[k]) < 1.0e-2)
			{
				sat_info_valid = 0;
				break;
			}
		}
		if (0 == sat_info_valid)
		{
			continue;
		}
		delta_dist = g_sat2site_dist[i];
		geodist_tdcp(sat_data->sat_pos_clk, ori_xyz, e);
		ele = (base_sat_data->ele)* D2R;
		sin_ele = sin(ele);
		if (ele > rad_30)
		{
			weight = 1.0;
		}
		else
		{
			weight = 4 * sin_ele * sin_ele;
		}
		for (k = 0; k < 1; ++k)
		{
			index = i* (NFREQ + NEXOBS) + k;
			if (wp[index] < 1.0e-3 || 0 == slip_tag[index])
			{
				continue;
			}
			if (0 == compare_code_consist(sys, k, sat_data, base_sat_data, &wave))
			{
				continue;
			}
			clearN();
			for (m = 0; m < 3; ++m)
			{
				addH(m, -e[m]);
			}
			addH(3 + sys_model, 1.0);
			omc = (sat_data->L[k] - base_sat_data->L[k]) * wave - delta_dist - rcv_clk_drift[sys_model];
			setOMC(omc, 1.0 / (weight * wp[index] + 1.0e-12));
			predictStep(delta_para, Q);
			measUpdate(X, delta_para, Q, 0.0);
			enter_filter_num++;
		}
	}
	//end
	freeSeqMatrix();
	//get sigma0
	sat_valid_num = 0;
	for (i = 0; i < GNSS_MAX_MODE; ++i)
	{
		sys_valid_obs[i] = 0;
	}
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		sys = satsys(sat_data->sat, &prn);
		sys_model = GNSS_SYS2MODE(sys);

		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		sat_info_valid = 1;
		for (k = 0; k < 3; ++k)
		{
			if (fabs(sat_data->sat_pos_clk[k]) < 1.0e-2 || fabs(base_sat_data->sat_pos_clk[k]) < 1.0e-2)
			{
				sat_info_valid = 0;
				break;
			}
		}
		if (0 == sat_info_valid)
		{
			continue;
		}
		base_sat_data = g_base_tdcp_data.sat_data + j;
		delta_dist = g_sat2site_dist[i];
		geodist_tdcp(sat_data->sat_pos_clk, ori_xyz, e);
		ele = (base_sat_data->ele) * D2R;
		sin_ele = sin(ele);
		if (ele > rad_30)
		{
			weight = 1.0;
		}
		else
		{
			weight = 4 * sin_ele * sin_ele;
		}
		sat_obs_valid = 0;
		res_less_005_num = 0;
		for (k = 0; k < 1; ++k)
		{
			index = i * (NFREQ + NEXOBS) + k;
			if (0 == compare_code_consist(sys, k, sat_data, base_sat_data, &wave))
			{
				continue;
			}
			if (0 != slip_tag[index])
			{
				omc = (sat_data->L[k] - base_sat_data->L[k]) * wave - delta_dist;
				for (m = 0; m < 3; ++m)
				{
					omc += e[m] * delta_para[m];
				}
				omc -= (delta_para[3 + sys_model] + rcv_clk_drift[sys_model]);
				*sigma0 += omc * weight*wp[index] * omc;
				++valid_obs_num;
				++sat_obs_valid;
				if (fabs(omc))
				{
					++res_less_005_num;
				}
			}
		}
		if (sat_obs_valid > 0)
		{
			++sat_valid_num;
			++sys_valid_obs[sys_model];
		}
		if (res_less_005_num > 0)
		{
			++(*res_less_005_sat);
		}
	}
	valid_sys_num = 0;
	for (i = 0; i < GNSS_MAX_MODE; ++i)
	{
		if (sys_valid_obs[i] > 0)
		{
			++valid_sys_num;
		}
	}
	*para_num = 3 + valid_sys_num;
	if (sat_valid_num <= 3+ valid_sys_num)
	{
		return 0;
	}
	*sigma0 = sqrt(*sigma0 / ((double)valid_obs_num - *para_num));
	//end
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		sys = satsys(sat_data->sat, &prn);
		sys_model = GNSS_SYS2MODE(sys);
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		sat_info_valid = 1;
		for (k = 0; k < 3; ++k)
		{
			if (fabs(sat_data->sat_pos_clk[k]) < 1.0e-2 || fabs(base_sat_data->sat_pos_clk[k]) < 1.0e-2)
			{
				sat_info_valid = 0;
				break;
			}
		}
		if (0 == sat_info_valid)
		{
			continue;
		}
		base_sat_data = g_base_tdcp_data.sat_data + j;
		delta_dist = g_sat2site_dist[i];
		geodist_tdcp(sat_data->sat_pos_clk, ori_xyz, e);
		for (k = 0; k < 1; ++k)
		{
			index = i * (NFREQ + NEXOBS) + k;
			if (0 == compare_code_consist(sys, k, sat_data, base_sat_data, &wave))
			{
				continue;
			}
			if (0 != slip_tag[index])
			{
				omc = (sat_data->L[k] - base_sat_data->L[k]) * wave - delta_dist - rcv_clk_drift[sys_model];
				for (m = 0; m < 3; ++m)
				{
					omc += e[m] * delta_para[m];
				}
				omc -= delta_para[3 + sys_model];
				wp[index] = get_wp(1.0, 3.0, *sigma0, omc);
			}
		}
	}

	for (i = 0; i < 3; ++i)
	{
		delta_x[i] = delta_para[i];
		cur_xyz[i] = ori_xyz[i] + delta_x[i];
	}
	if (enter_filter_num < *para_num + 2)
	{
		return 0;
	}
	return 1;
}
void cal_sat2site_info(const double ori_xyz[3])
{
	int i = 0, j = 0, k = 0/*, sys = 0, prn = 0, sys_model = 0*/, sat_info_valid = 0;
	asg_sat_tdcp* sat_data = NULL, * base_sat_data = NULL;
	double e[3] = { 0.0 };
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		//sys = satsys(sat_data->sat, &prn);
		//sys_model = GNSS_SYS2MODE(sys);
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		base_sat_data = g_base_tdcp_data.sat_data + j;
		sat_info_valid = 1;
		for (k = 0; k < 3; ++k)
		{
			if (fabs(sat_data->sat_pos_clk[k]) < 1.0e-2 || fabs(base_sat_data->sat_pos_clk[k]) < 1.0e-2)
			{
				sat_info_valid = 0;
				break;
			}
		}
		if (0 == sat_info_valid)
		{
			continue;
		}
		g_sat2site_dist[i] = get_site2sat_dist(sat_data, base_sat_data, ori_xyz, e);
		g_sat2site_unit[i][0] = e[0];
		g_sat2site_unit[i][1] = e[1]; 
		g_sat2site_unit[i][2] = e[2];
	}
	return;
}
int feedup_obs_tdcp_filter_float(double rcv_clk_drift,const double ori_xyz[3],unsigned char* slip_tag, double delta_x[3],float* wp,
	double* sigma0, int* good_quality_obs_num, int* para_num,double prior_sigma0)
{
	float Q[MAX_Q_ELE] = { 0.0 };
	float X[MAX_PARA_TDCP] = { 0.0 };
	float delta_para[MAX_PARA_TDCP] = { 0.0 };
	int i = 0, j = 0, k = 0, m = 0, enter_filter_num = 0;
	int sys = 0, prn = 0, index = 0;
	//double wave1 = 0.0, wave2 = 0.0, base_wave1 = 0.0, base_wave2 = 0.0;
	asg_sat_tdcp* sat_data = NULL, * base_sat_data = NULL;

	double wave = 0.0, delta_dist = 0.0, omc = 0.0/*, res = 0.0*/, max_res = 0.0;
	int sat_info_valid = 0, valid_obs_num = 0;
	int sat_obs_valid = 0, sat_valid_num = 0;
	int res_less_005_num = 0, max_res_index = -1;
	float ele = 0.0f, rad_30 = (float)(30.0 * D2R), sin_ele = 0.0f, weight = 1.0f;
	*sigma0 = 0.0;
	*good_quality_obs_num = 0;
	*para_num = 0;
	for (i = 0; i < 3; ++i)
	{
		Q[i * MAX_PARA_TDCP + i] = 1.0e6;
	}
	for (i = 3; i < MAX_PARA_TDCP; ++i)
	{
		Q[i * MAX_PARA_TDCP + i] = 1.0e4;
	}
	initH_f(MAX_PARA_TDCP);
	//carrier phase enter into kalman filter
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		sys = satsys(sat_data->sat, &prn);
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		if ((sat_data->LLI[0] & 0x2) == 0x2) continue;
		base_sat_data = g_base_tdcp_data.sat_data + j;
		sat_info_valid = 1;
		for (k = 0; k < 3; ++k)
		{
			if (fabs(sat_data->sat_pos_clk[k]) < 1.0e-2 || fabs(base_sat_data->sat_pos_clk[k]) < 1.0e-2)
			{
				sat_info_valid = 0;
				break;
			}
		}
		if (0 == sat_info_valid)
		{
			continue;
		}
		delta_dist = g_sat2site_dist[i];
		sat_data->azimuth = base_sat_data->azimuth;//obtain azimuth
		sat_data->ele = base_sat_data->ele;
		ele = (float)((base_sat_data->ele) * D2R);
		sin_ele = (float)sin(ele);
		if (ele > rad_30)
		{
			weight = 1.0;
		}
		else
		{
			weight = 4 * sin_ele * sin_ele;
		}
		for (k = 0; k < 1; ++k)
		{
			index = i * (NFREQ + NEXOBS) + k;
			if (wp[index] < 1.0e-3 || 0 == slip_tag[index])
			{
				continue;
			}
			if (0 == compare_code_consist(sys, k, sat_data, base_sat_data, &wave))
			{
				continue;
			}
			clearN_f();
			for (m = 0; m < 3; ++m)
			{
				addH_f(m, (float)-g_sat2site_unit[i][m]);
			}
			addH_f(3 + 0, 1.0);
			omc = (sat_data->L[k] - base_sat_data->L[k]) * wave - delta_dist - rcv_clk_drift;
			setOMC_f((float)omc, (float)(1.0 / ((double)weight * wp[index] + 1.0e-12)));
			predictStep_f(delta_para, Q);
			measUpdate_f(X, delta_para, Q, 0.0);
			enter_filter_num++;
		}
	}
	//end
	reset_seq_ekf_f();
	freeSeqMatrix_f();
	//get sigma0
	sat_valid_num = 0;
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		sys = satsys(sat_data->sat, &prn);

		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		if ((sat_data->LLI[0] & 0x2) == 0x2) continue;
		base_sat_data = g_base_tdcp_data.sat_data + j;

		sat_info_valid = 1;
		for (k = 0; k < 3; ++k)
		{
			if (fabs(sat_data->sat_pos_clk[k]) < 1.0e-2 || fabs(base_sat_data->sat_pos_clk[k]) < 1.0e-2)
			{
				sat_info_valid = 0;
				break;
			}
		}
		if (0 == sat_info_valid)
		{
			continue;
		}
		delta_dist = g_sat2site_dist[i];
		ele = (float)((base_sat_data->ele) * D2R);
		sin_ele = (float)sin(ele);
		if (ele > rad_30)
		{
			weight = 1.0;
		}
		else
		{
			weight = 4 * sin_ele * sin_ele;
		}
		sat_obs_valid = 0;
		res_less_005_num = 0;
		for (k = 0; k < 1; ++k)
		{
			index = i * (NFREQ + NEXOBS) + k;
			if (0 == compare_code_consist(sys, k, sat_data, base_sat_data, &wave))
			{
				continue;
			}
			if (0 != slip_tag[index])
			{
				omc = (sat_data->L[k] - base_sat_data->L[k]) * wave - delta_dist;
				for (m = 0; m < 3; ++m)
				{
					omc += g_sat2site_unit[i][m] * delta_para[m];
				}
				omc -= (delta_para[3] + rcv_clk_drift);
				if (fabs(omc) > max_res && wp[index] >= 1.0)
				{
					max_res = fabs(omc);
					max_res_index = index;
				}
				*sigma0 += omc * weight * wp[index] * omc;
				++valid_obs_num;
				++sat_obs_valid;
				if (fabs(omc)< prior_sigma0)
				{
					++res_less_005_num;
				}
			}
		}
		if (sat_obs_valid > 0)
		{
			++sat_valid_num;
		}
		if (res_less_005_num > 0)
		{
			++(*good_quality_obs_num);
		}
	}
	if (max_res_index >= 0)
	{
		wp[max_res_index] = get_wp(1.0, 8.0, prior_sigma0, max_res);
	}
	*para_num = 3 + 1;
	if (sat_valid_num <= (*para_num))
	{
		return 0;
	}
	*sigma0 = sqrt(*sigma0 / ((double)valid_obs_num - *para_num));
	//end
	for (i = 0; i < 3; ++i)
	{
		delta_x[i] = delta_para[i];
	}
	if (enter_filter_num < *para_num + 2)
	{
		return 0;
	}
	return 1;
}
int feedup_obs_tdpr_filter_float(double rcv_clk_drift, const double ori_xyz[3], unsigned char* pseudo_gross_tag, double delta_x[3], float* wp,
	double* sigma0, int* good_quality_obs_num, int* para_num, double prior_sigma0)
{
	float Q[MAX_Q_ELE] = { 0.0 };
	float X[MAX_PARA_TDCP] = { 0.0 };
	float delta_para[MAX_PARA_TDCP] = { 0.0 };
	int i = 0, j = 0, k = 0, m = 0, enter_filter_num = 0;
	int sys = 0, prn = 0, index = 0;
	//double wave1 = 0.0, wave2 = 0.0, base_wave1 = 0.0, base_wave2 = 0.0;
	asg_sat_tdcp* sat_data = NULL, * base_sat_data = NULL;

	double delta_dist = 0.0, omc = 0.0, /*res = 0.0, wave = 0.0,*/ max_res = 0.0;
	int sat_info_valid = 0, valid_obs_num = 0;
	int sat_obs_valid = 0, sat_valid_num = 0;
	int res_less_005_num = 0, max_res_index = -1;
	float ele = 0.0f, rad_30 = (float)(30.0 * D2R), sin_ele = 0.0f, weight = 1.0f;
	*sigma0 = 0.0;
	*good_quality_obs_num = 0;
	*para_num = 0;
	for (i = 0; i < 3; ++i)
	{
		Q[i * MAX_PARA_TDCP + i] = 1.0e6;
	}
	for (i = 3; i < MAX_PARA_TDCP; ++i)
	{
		Q[i * MAX_PARA_TDCP + i] = 1.0e4;
	}
	initH_f(MAX_PARA_TDCP);
	//carrier phase enter into kalman filter
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		sys = satsys(sat_data->sat, &prn);
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		base_sat_data = g_base_tdcp_data.sat_data + j;
		sat_info_valid = 1;
		for (k = 0; k < 3; ++k)
		{
			if (fabs(sat_data->sat_pos_clk[k]) < 1.0e-2 || fabs(base_sat_data->sat_pos_clk[k]) < 1.0e-2)
			{
				sat_info_valid = 0;
				break;
			}
		}
		if (0 == sat_info_valid)
		{
			continue;
		}
		delta_dist = g_sat2site_dist[i];
		ele = (float)((base_sat_data->ele) * D2R);
		sin_ele = (float)sin(ele);
		if (ele > rad_30)
		{
			weight = 1.0;
		}
		else
		{
			weight = 4 * sin_ele * sin_ele;
		}
		for (k = 0; k < 1; ++k)
		{
			index = i * (NFREQ + NEXOBS) + k;
			if (wp[index] < 1.0e-3 || (0 == pseudo_gross_tag[index]))
			{
				continue;
			}
			if (0 == compare_code_consist(sys, k, sat_data, base_sat_data, NULL))
			{
				continue;
			}
			clearN_f();
			for (m = 0; m < 3; ++m)
			{
				addH_f(m, (float)-g_sat2site_unit[i][m]);
			}
			addH_f(3, 1.0);
			omc = (sat_data->P[k] - base_sat_data->P[k]) - delta_dist - rcv_clk_drift;
			setOMC_f((float)omc, (float)(1.0 / ((double)weight * wp[index] + 1.0e-12)));
			predictStep_f(delta_para, Q);
			measUpdate_f(X, delta_para, Q, 0.0);
			enter_filter_num++;
		}
	}
	//end
	reset_seq_ekf_f();
	freeSeqMatrix_f();
	//get sigma0
	sat_valid_num = 0;
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		sys = satsys(sat_data->sat, &prn);

		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		base_sat_data = g_base_tdcp_data.sat_data + j;

		sat_info_valid = 1;
		for (k = 0; k < 3; ++k)
		{
			if (fabs(sat_data->sat_pos_clk[k]) < 1.0e-2 || fabs(base_sat_data->sat_pos_clk[k]) < 1.0e-2)
			{
				sat_info_valid = 0;
				break;
			}
		}
		if (0 == sat_info_valid)
		{
			continue;
		}
		delta_dist = g_sat2site_dist[i];
		ele = (float)((base_sat_data->ele) * D2R);
		sin_ele = (float)sin(ele);
		if (ele > rad_30)
		{
			weight = 1.0;
		}
		else
		{
			weight = 4 * sin_ele * sin_ele;
		}
		sat_obs_valid = 0;
		res_less_005_num = 0;
		for (k = 0; k < 1; ++k)
		{
			index = i * (NFREQ + NEXOBS) + k;
			if (0 == compare_code_consist(sys, k, sat_data, base_sat_data, NULL))
			{
				continue;
			}
			if (0 != pseudo_gross_tag[index])
			{
				omc = (sat_data->P[k] - base_sat_data->P[k]) - delta_dist;
				for (m = 0; m < 3; ++m)
				{
					omc += g_sat2site_unit[i][m] * delta_para[m];
				}
				omc -= (delta_para[3] + rcv_clk_drift);
				if (fabs(omc) > max_res && wp[index] >= 1.0)
				{
					max_res = fabs(omc);
					max_res_index = index;
				}
				*sigma0 += omc * weight * wp[index] * omc;
				++valid_obs_num;
				++sat_obs_valid;
				if (fabs(omc) < 1.5)
				{
					++res_less_005_num;
				}
			}
		}
		if (sat_obs_valid > 0)
		{
			++sat_valid_num;
		}
		if (res_less_005_num > 0)
		{
			++(*good_quality_obs_num);
		}
	}
	if (max_res_index >= 0)
	{
		wp[max_res_index] = get_wp(1.0, 8.0, prior_sigma0, max_res);
	}
	*para_num = 3 + 1;
	if (sat_valid_num <= 3 + 1)
	{
		return 0;
	}
	*sigma0 = sqrt(*sigma0 / ((double)valid_obs_num - *para_num));
	//end
	for (i = 0; i < 3; ++i)
	{
		delta_x[i] = delta_para[i];
	}
	if (enter_filter_num < *para_num + 2)
	{
		return 0;
	}
	return 1;
}
int feedup_obs_tddr_filter_float(double rcv_clk_drift, const double ori_xyz[3], unsigned char* pseudo_gross_tag, double delta_x[3], float* wp,
	double* sigma0, int* good_quality_obs_num, int* para_num, double prior_sigma0)
{
	float Q[MAX_Q_ELE] = { 0.0 };
	float X[MAX_PARA_TDCP] = { 0.0 };
	float delta_para[MAX_PARA_TDCP] = { 0.0 };
	int i = 0, j = 0, k = 0, m = 0, enter_filter_num = 0;
	int sys = 0, prn = 0, index = 0;
	//double wave1 = 0.0, wave2 = 0.0, base_wave1 = 0.0, base_wave2 = 0.0;
	asg_sat_tdcp* sat_data = NULL, * base_sat_data = NULL;

	double delta_dist = 0.0, omc = 0.0/*, res = 0.0, wave = 0.0*/, max_res = 0.0;
	int sat_info_valid = 0, valid_obs_num = 0;
	int sat_obs_valid = 0, sat_valid_num = 0;
	int res_less_005_num = 0, max_res_index = -1;
	float ele = 0.0f, rad_30 = (float)(30.0 * D2R), sin_ele = 0.0f, weight = 1.0f;
	double delta_t = asg_timediff(g_cur_epoch_data.obs_time, g_base_tdcp_data.obs_time);
	double avg_doppler = 0.0;
	*sigma0 = 0.0;
	*good_quality_obs_num = 0;
	*para_num = 0;
	for (i = 0; i < 3; ++i)
	{
		Q[i * MAX_PARA_TDCP + i] = 1.0e6;
	}
	for (i = 3; i < MAX_PARA_TDCP; ++i)
	{
		Q[i * MAX_PARA_TDCP + i] = 1.0e4;
	}
	initH_f(MAX_PARA_TDCP);
	//carrier phase enter into kalman filter
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		sys = satsys(sat_data->sat, &prn);
		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		base_sat_data = g_base_tdcp_data.sat_data + j;
		sat_info_valid = 1;
		for (k = 0; k < 3; ++k)
		{
			if (fabs(sat_data->sat_pos_clk[k]) < 1.0e-2 || fabs(base_sat_data->sat_pos_clk[k]) < 1.0e-2)
			{
				sat_info_valid = 0;
				break;
			}
		}
		if (0 == sat_info_valid)
		{
			continue;
		}
		delta_dist = g_sat2site_dist[i];
		ele = (float)((base_sat_data->ele) * D2R);
		sin_ele = (float)sin(ele);
		if (ele > rad_30)
		{
			weight = 1.0;
		}
		else
		{
			weight = 4 * sin_ele * sin_ele;
		}
		for (k = 0; k < 1; ++k)
		{
			index = i * (NFREQ + NEXOBS) + k;
			if (wp[index] < 1.0e-3 || (0 == pseudo_gross_tag[index]))
			{
				continue;
			}
			if (0 == compare_code_consist(sys, k, sat_data, base_sat_data, NULL))
			{
				continue;
			}
			clearN_f();
			for (m = 0; m < 3; ++m)
			{
				addH_f(m, (float)-g_sat2site_unit[i][m]);
			}
			addH_f(3, 1.0);
			avg_doppler = 0.0;
			if (fabs(sat_data->D[k]) > 0.0 && fabs(base_sat_data->D[k]) > 0.0)
			{
				avg_doppler = ((double)sat_data->D[k] + base_sat_data->D[k]) * 0.5;
			}
			else if (fabs(sat_data->D[k]) > 0.0)
			{
				avg_doppler = sat_data->D[k];
			}
			else if (fabs(base_sat_data->D[k]) > 0.0)
			{
				avg_doppler = base_sat_data->D[k];
			}
			omc = avg_doppler*delta_t - delta_dist - rcv_clk_drift;
			setOMC_f((float)omc, (float)(1.0 / ((double)weight * wp[index] + 1.0e-12)));
			predictStep_f(delta_para, Q);
			measUpdate_f(X, delta_para, Q, 0.0);
			enter_filter_num++;
		}
	}
	//end
	reset_seq_ekf_f();
	freeSeqMatrix_f();
	//get sigma0
	sat_valid_num = 0;
	for (i = 0; i < g_cur_epoch_data.sat_num; ++i)
	{
		sat_data = g_cur_epoch_data.sat_data + i;
		sys = satsys(sat_data->sat, &prn);

		base_sat_data = g_base_tdcp_data.sat_data;
		for (j = 0; j < g_base_tdcp_data.sat_num; ++j)
		{
			if (base_sat_data[j].sat == sat_data->sat)
			{
				break;
			}
		}
		if (j == g_base_tdcp_data.sat_num)
		{
			continue;
		}
		base_sat_data = g_base_tdcp_data.sat_data + j;

		sat_info_valid = 1;
		for (k = 0; k < 3; ++k)
		{
			if (fabs(sat_data->sat_pos_clk[k]) < 1.0e-2 || fabs(base_sat_data->sat_pos_clk[k]) < 1.0e-2)
			{
				sat_info_valid = 0;
				break;
			}
		}
		if (0 == sat_info_valid)
		{
			continue;
		}
		delta_dist = g_sat2site_dist[i];
		ele = (float)((base_sat_data->ele) * D2R);
		sin_ele = (float)sin(ele);
		if (ele > rad_30)
		{
			weight = 1.0;
		}
		else
		{
			weight = 4 * sin_ele * sin_ele;
		}
		sat_obs_valid = 0;
		res_less_005_num = 0;
		for (k = 0; k < 1; ++k)
		{
			index = i * (NFREQ + NEXOBS) + k;
			if (0 == compare_code_consist(sys, k, sat_data, base_sat_data, NULL))
			{
				continue;
			}
			if (0 != pseudo_gross_tag[index])
			{
				avg_doppler = 0.0;
				if (fabs(sat_data->D[k]) > 0.0 && fabs(base_sat_data->D[k]) > 0.0)
				{
					avg_doppler = ((double)sat_data->D[k] + base_sat_data->D[k]) * 0.5;
				}
				else if (fabs(sat_data->D[k]) > 0.0)
				{
					avg_doppler = sat_data->D[k];
				}
				else if (fabs(base_sat_data->D[k]) > 0.0)
				{
					avg_doppler = base_sat_data->D[k];
				}
				omc = avg_doppler*delta_t - delta_dist;
				for (m = 0; m < 3; ++m)
				{
					omc += g_sat2site_unit[i][m] * delta_para[m];
				}
				omc -= (delta_para[3] + rcv_clk_drift);
				if (fabs(omc) > max_res && wp[index] >= 1.0)
				{
					max_res = fabs(omc);
					max_res_index = index;
				}
				*sigma0 += omc * weight * wp[index] * omc;
				++valid_obs_num;
				++sat_obs_valid;
				if (fabs(omc) < prior_sigma0)
				{
					++res_less_005_num;
				}
			}
		}
		if (sat_obs_valid > 0)
		{
			++sat_valid_num;
		}
		if (res_less_005_num > 0)
		{
			++(*good_quality_obs_num);
		}
	}
	if (max_res_index >= 0)
	{
		wp[max_res_index] = get_wp(1.0, 8.0, prior_sigma0, max_res);
	}
	*para_num = 3 + 1;
	if (sat_valid_num <= 3 + 1)
	{
		return 0;
	}
	*sigma0 = sqrt(*sigma0 / ((double)valid_obs_num - *para_num));
	//end
	for (i = 0; i < 3; ++i)
	{
		delta_x[i] = delta_para[i];
	}
	if (enter_filter_num < *para_num + 2)
	{
		return 0;
	}
	return 1;
}
void fill_nmea(asensing_tdcp_pos_t* rover_tdcp_pos)
{
	memset(rover_tdcp_pos->nmea_gga, 0, sizeof(rover_tdcp_pos->nmea_gga));
	memset(rover_tdcp_pos->nmea_rmc, 0, sizeof(rover_tdcp_pos->nmea_rmc));
	gnss_nmea_result_t user_nmea = { 0 };
	user_nmea.timestamp = rover_tdcp_pos->utc_timestamp;
	user_nmea.latitude = rover_tdcp_pos->lat*D2R;
	user_nmea.longitude = rover_tdcp_pos->lon*D2R;
	user_nmea.altitude = rover_tdcp_pos->alt;
	user_nmea.size = sizeof(gnss_nmea_result_t);
	user_nmea.flags = 0;
	user_nmea.speed = sqrt(((double)rover_tdcp_pos->vel_e * rover_tdcp_pos->vel_e)
		+ ((double)rover_tdcp_pos->vel_n * rover_tdcp_pos->vel_n)) / NMEA_VEL_KNOT;
	user_nmea.speed = user_nmea.speed < 999.9 ? user_nmea.speed : 999.9;
	user_nmea.bearing = 0.0;
	user_nmea.accuracy = 0.0;
	user_nmea.time_flag = 1;
	user_nmea.year = rover_tdcp_pos->utc_year;
	user_nmea.month = rover_tdcp_pos->utc_month;
	user_nmea.day = rover_tdcp_pos->utc_day;
	user_nmea.hour = rover_tdcp_pos->utc_hour;
	user_nmea.minute = rover_tdcp_pos->utc_minute;
	user_nmea.second = rover_tdcp_pos->utc_second;
	user_nmea.PDOP = rover_tdcp_pos->pdop;
	user_nmea.HDOP = rover_tdcp_pos->hdop;
	user_nmea.VDOP = rover_tdcp_pos->vdop;
	user_nmea.TDOP = rover_tdcp_pos->tdop;
	/* check HDOP value if in abnormal condition */
	if (user_nmea.HDOP > 99.99f)
	{
		user_nmea.HDOP = 99.99f;
	}
	else if (user_nmea.HDOP < 0.0f)
	{
		user_nmea.HDOP = 0.0f;
	}
	if (0 == rover_tdcp_pos->fix_quality)
	{
		user_nmea.indicator = GNSS_NMEA_QUALITY_INVALID;
	}
	else if (1 == rover_tdcp_pos->fix_quality)
	{
		user_nmea.indicator = GNSS_NMEA_QUALITY_GNSS_SPS;
	}
	else if (2 == rover_tdcp_pos->fix_quality)
	{
		user_nmea.indicator = GNSS_NMEA_QUALITY_DGNSS_SPS;
	}
	else if (5 == rover_tdcp_pos->fix_quality)
	{
		user_nmea.indicator = GNSS_NMEA_QUALITY_RTK_FLOATING;
	}
	else if (4 == rover_tdcp_pos->fix_quality)
	{
		user_nmea.indicator = GNSS_NMEA_QUALITY_RTK_FIXED;
	}
	else if (6 == rover_tdcp_pos->fix_quality)
	{
		user_nmea.indicator = GNSS_NMEA_QUALITY_DR;
	}
	user_nmea.sv_inuesd = rover_tdcp_pos->sv_used;
	user_nmea.ageOfDiff = rover_tdcp_pos->age;
	user_nmea.ve = rover_tdcp_pos->vel_e;
	user_nmea.vn = rover_tdcp_pos->vel_n;
	user_nmea.vu = -(rover_tdcp_pos->vel_d);
#if defined(GNSS_NMEA_ENABLE_GSV) && defined(ENABLE_GSV_GST)
	int i, sys = 0, sys_model = 0, prn = 0, sat_num = 0;
	asg_sat_tdcp* sat_data = NULL;

	for (i = 0;i < g_cur_epoch_data.sat_num; i++)
	{
		if (i >= GNSS_NMEA_MAX_MEASUREMENT)
		{
			break;
		}

		sat_data = g_cur_epoch_data.sat_data + i;
		sys = satsys(sat_data->sat, &prn);
		sys_model = GNSS_SYS2MODE(sys);

		user_nmea.sv_list[i].size = sizeof(gnss_nmea_sv_info);

		switch (sys_model)
		{
		case GPS_MODE:
			if (prn < 40)
			{
				user_nmea.sv_list[i].constellation = GNSS_NMEA_CONSTELLATION_GPS;
				user_nmea.sv_list[i].svid = prn;
			}
			else
			{
				user_nmea.sv_list[i].constellation = GNSS_NMEA_CONSTELLATION_QZSS;
				user_nmea.sv_list[i].svid = prn;
			}
			break;
		case GLN_MODE:
			user_nmea.sv_list[i].constellation = GNSS_NMEA_CONSTELLATION_GLONASS;
			user_nmea.sv_list[i].svid = prn;
			break;
		case BDS_MODE:
			user_nmea.sv_list[i].constellation = GNSS_NMEA_CONSTELLATION_BEIDOU;
			user_nmea.sv_list[i].svid = prn;
			break;
		case GAL_MODE:
			user_nmea.sv_list[i].constellation = GNSS_NMEA_CONSTELLATION_GALILEO;
			user_nmea.sv_list[i].svid = prn;
			break;
		default:break;
		}
		user_nmea.sv_list[i].c_n0_dbhz = sat_data->SNR[0];
		user_nmea.sv_list[i].elevation = (float)sat_data->ele;
		user_nmea.sv_list[i].azimuth = sat_data->azimuth;

		if (user_nmea.sv_list[i].c_n0_dbhz > 25.0)
		{
			user_nmea.sv_list[i].flags = GNSS_NMEA_SV_FLAGS_USED_IN_FIX;
		}
	}

	user_nmea.num_svs = (g_cur_epoch_data.sat_num < GNSS_NMEA_MAX_MEASUREMENT) ? g_cur_epoch_data.sat_num : GNSS_NMEA_MAX_MEASUREMENT;

	{
		gnss_nmea_sv_info temp = { 0 };
		int isChange = 0;
		int len = user_nmea.num_svs;

		for (int i = 0; i < len - 1; i++)
		{
			isChange = 0;

			for (int j = 0; j < len - i - 1; j++)
			{
				int charge = 0;

				if (user_nmea.sv_list[j].constellation > user_nmea.sv_list[j + 1].constellation)
				{
					charge = 1;
				}
				else if (user_nmea.sv_list[j].constellation == user_nmea.sv_list[j + 1].constellation)
				{
					if (user_nmea.sv_list[j].svid > user_nmea.sv_list[j + 1].svid)
					{
						charge = 1;
					}
				}

				if (charge)
				{
					memcpy(&temp, &user_nmea.sv_list[j], sizeof(gnss_nmea_sv_info));
					memcpy(&user_nmea.sv_list[j], &user_nmea.sv_list[j + 1], sizeof(gnss_nmea_sv_info));
					memcpy(&user_nmea.sv_list[j + 1], &temp, sizeof(gnss_nmea_sv_info));
					isChange = 1;
				}
			}

			if (!isChange)
			{
				break;
			}
		}
	}
#endif
	gnss_nmea_extract_gga(&user_nmea, rover_tdcp_pos->nmea_gga, &(rover_tdcp_pos->quasi_geoid_h));
	gnss_nmea_extract_rmc(&user_nmea, rover_tdcp_pos->nmea_rmc);

#ifdef ENABLE_GSV_GST
	gnss_nmea_extract_gsv(&user_nmea, rover_tdcp_pos->nmea_gsv, &rover_tdcp_pos->nmea_gsv_num, sizeof(rover_tdcp_pos->nmea_gsv) / sizeof(rover_tdcp_pos->nmea_gsv[0]));
	gnss_nmea_extract_gst_bg(&user_nmea, rover_tdcp_pos->nmea_gst);
#endif

#if defined(PLAYBACK_MODE) && defined(ENABLE_GSV_GST)
	char nmea_buf[256] = { 0 };
	for (int i = 0; i < rover_tdcp_pos->nmea_gsv_num; i++) {
		memset(nmea_buf, 0, sizeof(nmea_buf));
		strncpy(nmea_buf, rover_tdcp_pos->nmea_gsv[i], strlen(rover_tdcp_pos->nmea_gsv[i]) - 2);
		//GLOGI(nmea_buf);
	}

	if (rover_tdcp_pos->nmea_gst[0] != '\0')
	{
		memset(nmea_buf, 0, sizeof(nmea_buf));
		strncpy(nmea_buf, rover_tdcp_pos->nmea_gst, strlen(rover_tdcp_pos->nmea_gst) - 2);
		//GLOGI(nmea_buf);
	}	
#endif

	return;
}
int getTDCPpos(const double ori_xyz[3],float* wp, const unsigned char* slip_tag, int total_num, double delta_x[3], double cur_xyz[3])
{
	int i = 0, norm_num = 0, max_iter = 10, filter_status = 0, j = 0, is_continued = 1;
	int good_quality_obs_num = 0, para_num = 0;
	double rcv_clk_drift = 0.0, dist_diff = 0.0, sigma0 = 0.0, pre_delta_x[3] = { 0.0 };
	norm_num += getRcvClkDriftTDCP((unsigned char *)slip_tag, &rcv_clk_drift);
	for (i = 0; i < total_num; ++i)
	{
		wp[i] = 1.0;
	}
	for (i = 0; i < max_iter; ++i)
	{
		good_quality_obs_num = 0;
		para_num = 0;
		filter_status = feedup_obs_tdcp_filter_float(rcv_clk_drift, ori_xyz, (unsigned char *)slip_tag, delta_x, wp, &sigma0,
													 &good_quality_obs_num, &para_num, 0.05);
		dist_diff = 0.0;
		for (j = 0; j < 3; ++j)
		{
			dist_diff += (delta_x[j] - pre_delta_x[j]) * (delta_x[j] - pre_delta_x[j]);
			pre_delta_x[j] = delta_x[j];
		}
		is_continued = 0;
		for (j = 0; j < total_num; ++j)
		{
			if (wp[j] < 1.0)
			{
				is_continued = 1;
				break;
			}
		}
		if (dist_diff < 1.0e-4 || 0 == is_continued)
		{
			break;
		}
	}
	for (j = 0; j < 3; ++j)
	{
		cur_xyz[j] = ori_xyz[j] + delta_x[j];
	}
	if (norm_num < para_num + 2 || sigma0 > 0.2 || i == max_iter ||
		good_quality_obs_num < para_num)
	{
		filter_status = 0;
	}
	return filter_status;
}
int getTDDRpos(const double ori_xyz[3], float* wp, int total_num, double delta_x[3], double cur_xyz[3])
{
	int i = 0, norm_num = 0, max_iter = 10, filter_status = 0, j = 0, is_continued = 1;
	int good_quality_obs_num = 0, para_num = 0;
	double rcv_clk_drift = 0.0, dist_diff = 0.0, sigma0 = 0.0, pre_delta_x[3] = { 0.0 };
	unsigned char pseudo_gross_tag[(NFREQ + NEXOBS) * MAXOBS] = { 0 };
	for (i = 0; i < total_num; ++i)
	{
		pseudo_gross_tag[i] = 0;
	}
	for (i = 0; i < NFREQ + NEXOBS; ++i)
	{
		doppler_detect_pseudo_range(pseudo_gross_tag, i);
	}
	norm_num += getRcvClkDriftTDDR(pseudo_gross_tag, &rcv_clk_drift);
	for (i = 0; i < total_num; ++i)
	{
		wp[i] = 1.0;
	}
	for (i = 0; i < max_iter; ++i)
	{
		good_quality_obs_num = 0;
		para_num = 0;
		filter_status = feedup_obs_tddr_filter_float(rcv_clk_drift, ori_xyz, pseudo_gross_tag, delta_x, wp, &sigma0,
			&good_quality_obs_num, &para_num, 3.0);
		dist_diff = 0.0;
		for (j = 0; j < 3; ++j)
		{
			dist_diff += (delta_x[j] - pre_delta_x[j]) * (delta_x[j] - pre_delta_x[j]);
			pre_delta_x[j] = delta_x[j];
		}
		is_continued = 0;
		for (j = 0; j < total_num; ++j)
		{
			if (wp[j] < 1.0)
			{
				is_continued = 1;
				break;
			}
		}
		if (dist_diff < 1.0e-4 || 0 == is_continued)
		{
			break;
		}
	}
	for (j = 0; j < 3; ++j)
	{
		cur_xyz[j] = ori_xyz[j] + delta_x[j];
	}
	if (norm_num < para_num + 2 || sigma0 > 3.0 || i == max_iter ||
		good_quality_obs_num < para_num)
	{
		filter_status = 0;
	}
	return filter_status;
}
int getTDPRpos(const double ori_xyz[3], float* wp, int total_num, double delta_x[3], double cur_xyz[3])
{
	int i = 0, norm_num = 0, max_iter = 10, filter_status = 0, j = 0, is_continued = 1;
	int good_quality_obs_num = 0, para_num = 0;
	double rcv_clk_drift = 0.0, dist_diff = 0.0, sigma0 = 0.0, pre_delta_x[3] = { 0.0 };
	unsigned char pseudo_gross_tag[(NFREQ + NEXOBS) * MAXOBS] = { 0 };
	for (i = 0; i < total_num; ++i)
	{
		pseudo_gross_tag[i] = 0;
	}
	for (i = 0; i < NFREQ + NEXOBS; ++i)
	{
		doppler_detect_pseudo_range(pseudo_gross_tag, i);
	}
	norm_num += getRcvClkDriftTDPR(pseudo_gross_tag, &rcv_clk_drift);
	for (i = 0; i < total_num; ++i)
	{
		wp[i] = 1.0;
	}
	for (i = 0; i < max_iter; ++i)
	{
		good_quality_obs_num = 0;
		para_num = 0;
		filter_status = feedup_obs_tdpr_filter_float(rcv_clk_drift, ori_xyz, pseudo_gross_tag, delta_x, wp, &sigma0,
			&good_quality_obs_num, &para_num, 3.0);
		dist_diff = 0.0;
		for (j = 0; j < 3; ++j)
		{
			dist_diff += (delta_x[j] - pre_delta_x[j]) * (delta_x[j] - pre_delta_x[j]);
			pre_delta_x[j] = delta_x[j];
		}
		is_continued = 0;
		for (j = 0; j < total_num; ++j)
		{
			if (wp[j] < 1.0)
			{
				is_continued = 1;
				break;
			}
		}
		if (dist_diff < 1.0e-4 || 0 == is_continued)
		{
			break;
		}
	}
	for (j = 0; j < 3; ++j)
	{
		cur_xyz[j] = ori_xyz[j] + delta_x[j];
	}
	if (norm_num < para_num + 2 || sigma0 > 3.0 || i == max_iter ||
		good_quality_obs_num < para_num)
	{
		filter_status = 0;
	}
	return filter_status;
}
void cleanPos(asensing_tdcp_pos_t* rover_tdcp_pos)
{
	rover_tdcp_pos->fix_quality = 0;
	rover_tdcp_pos->lat = 0.0;
	rover_tdcp_pos->lon = 0.0;
	rover_tdcp_pos->alt = 0.0;
	rover_tdcp_pos->latstd = 0.0;
	rover_tdcp_pos->lonstd = 0.0;
	rover_tdcp_pos->altstd = 0.0;

	rover_tdcp_pos->vel_e = 0.0;
	rover_tdcp_pos->vel_n = 0.0;
	rover_tdcp_pos->vel_d = 0.0;
	rover_tdcp_pos->vel_e_std = 0.0;
	rover_tdcp_pos->vel_n_std = 0.0;
	rover_tdcp_pos->vel_d_std = 0.0;

	rover_tdcp_pos->age = 0;
	rover_tdcp_pos->sv_used = 0;
	rover_tdcp_pos->sv_fixed = 0;

	rover_tdcp_pos->gdop = 0.0;
	rover_tdcp_pos->hdop = 0.0;
	rover_tdcp_pos->vdop = 0.0;
	rover_tdcp_pos->tdop = 0.0;
}
int getRealTimePos(asensing_tdcp_pos_t* rover_tdcp_pos)
{
	int i = 0, j = 0, week = 0, tdcp_status = 0, tdpr_status = 0;
	unsigned char slip_tag[(NFREQ + NEXOBS) * MAXOBS] = { 0 };
	float wp[(NFREQ + NEXOBS) * MAXOBS] = { 0.0 };
	double delta_x[3] = { 0.0 }, delta_v[3] = { 0.0 }, cur_xyz[3] = { 0.0 }, cur_blh[3] = { 0.0 }, cur_v_enu[3] = { 0.0 };
	double eph[6] = { 0.0 }, tow = 0.0;
	asg_gtime_t cur_time = g_cur_epoch_data.obs_time;
	asg_gtime_t pre_time = g_base_tdcp_data.obs_time;
	double delta_time = asg_timediff(cur_time, pre_time);
	int total_num = (NFREQ + NEXOBS) * MAXOBS;
	double ori_xyz[3] = { 0.0 };
	pos2ecef(g_base_tdcp_data.site_blh, ori_xyz);
	memset(rover_tdcp_pos, 0, sizeof(asensing_tdcp_pos_t));
	for (i = 0; i < MAXOBS; ++i)
	{
		for (j = 0; j < 3; ++j)
		{
			g_sat2site_unit[i][j] = 0.0;
		}
	}
	if (0 != g_base_tdcp_data.fix_quality)
	{
		*rover_tdcp_pos = g_base_pos;
		tow = rtklib_time2gpst(cur_time, &week);
		asg_gtime_t cur_utc_time = asg_gpst2utc(cur_time);
		asg_time2epoch(cur_utc_time, eph);
		rover_tdcp_pos->utc_timestamp = (long long)((long long)cur_utc_time.time * 1000 + cur_utc_time.sec * 1000);
		rover_tdcp_pos->utc_year = (uint16_t)(eph[0] + 0.5);
		rover_tdcp_pos->utc_month = (uint16_t)(eph[1] + 0.5);
		rover_tdcp_pos->utc_day = (uint16_t)(eph[2] + 0.5);
		rover_tdcp_pos->utc_hour = (uint16_t)(eph[3] + 0.5);
		rover_tdcp_pos->utc_minute = (uint16_t)(eph[4] + 0.5);
		rover_tdcp_pos->utc_second = eph[5];
		rover_tdcp_pos->week = week;
		rover_tdcp_pos->tow = tow;

		if (rover_tdcp_pos->age > 0.0)
		{
			rover_tdcp_pos->age += (float)delta_time;
		}
	}
	if (g_base_tdcp_data.sat_num <= 4 || 0 == g_base_tdcp_data.fix_quality || fabs(delta_time) >= 30.0 || fabs(delta_time)<1.0e-3
		|| (fabs(g_base_tdcp_data.site_blh[0])<1.0e-3 && fabs(g_base_tdcp_data.site_blh[1]) < 1.0e-3
		&& fabs(g_base_tdcp_data.site_blh[2]) < 1.0e-3))
	{
		cleanPos(rover_tdcp_pos);
		fill_nmea(rover_tdcp_pos);
		return 0;
	}
	fillup_rover_sat_info(&cur_time);
	corse_detect_slip(slip_tag);
	memset(g_sat2site_dist, 0, sizeof(g_sat2site_dist));
	cal_sat2site_info(ori_xyz);
	tdcp_status = getTDCPpos(ori_xyz, wp, slip_tag, total_num, delta_x, cur_xyz);
	if (0 == tdcp_status)
	{
		if (fabs(delta_time) < 10.0)
		{
			tdpr_status = getTDDRpos(ori_xyz, wp, total_num, delta_x, cur_xyz);
		}
		else
		{
			tdpr_status = getTDPRpos(ori_xyz, wp, total_num, delta_x, cur_xyz);
		}
		if (tdpr_status)
		{
			if (4 == (rover_tdcp_pos->fix_quality))
			{
				rover_tdcp_pos->fix_quality = 5;
			}
			else if (5 == (rover_tdcp_pos->fix_quality))
			{
				rover_tdcp_pos->fix_quality = 2;
			}
		}
	}
	if (tdcp_status || tdpr_status)
	{
		for (i = 0; i < 3; ++i)
		{
			delta_v[i] = delta_x[i] / delta_time;
		}
		ecef2pos(cur_xyz, cur_blh);
		ecef2enu(cur_blh, delta_v, cur_v_enu);

		rover_tdcp_pos->lat = cur_blh[0] * R2D;
		rover_tdcp_pos->lon = cur_blh[1] * R2D;
		rover_tdcp_pos->alt = (float)cur_blh[2];
		rover_tdcp_pos->vel_e = (float)cur_v_enu[0];
		rover_tdcp_pos->vel_n = (float)cur_v_enu[1];
		rover_tdcp_pos->vel_d = -(float)cur_v_enu[2];
	}
	else
	{
		rover_tdcp_pos->fix_quality = 0;
		rover_tdcp_pos->lat = 0;
		rover_tdcp_pos->lon = 0;
		rover_tdcp_pos->alt = 0;
		rover_tdcp_pos->latstd = 0;
		rover_tdcp_pos->lonstd = 0;
		rover_tdcp_pos->altstd = 0;

		rover_tdcp_pos->vel_e = 0;
		rover_tdcp_pos->vel_n = 0;
		rover_tdcp_pos->vel_d = 0;
		rover_tdcp_pos->vel_e_std = 0;
		rover_tdcp_pos->vel_n_std = 0;
		rover_tdcp_pos->vel_d_std = 0;

		rover_tdcp_pos->age = 0;
		rover_tdcp_pos->sv_used = 0;
		rover_tdcp_pos->sv_fixed = 0;

		rover_tdcp_pos->gdop = 0;
		rover_tdcp_pos->hdop = 0;
		rover_tdcp_pos->vdop = 0;
		rover_tdcp_pos->tdop = 0;
	}
	fill_nmea(rover_tdcp_pos);
	return 1;
}