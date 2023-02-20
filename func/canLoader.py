import numpy as np
import cantools
import can
from collections import defaultdict
import time
import os
from scipy.io import savemat
import json


class canLoader(object):
    """
    load blf data into mat by dbc(测试阶段)
    可参考：https://python-can.readthedocs.io/en/master/_modules/index.html
    @author: liqianwen
    @version: 0.0
    :param blf_dirs: 输入为文件名/文件夹名/dict={'name':path,...}
    :param dbc_dirs: 输入为文件名/文件夹名/dict={'name':path,...}
    """

    def __init__(self, blf_dirs='', dbc_dirs=''):
        self.blf_files_dict = {}
        self.dbc_files_dict = {}
        self.signals_dic = {}

        self.blf_dirs = blf_dirs
        self.dbc_dirs = dbc_dirs
        self.output_info = ''

    def add_dbc(self):
        """
        get dbc file path
        :return:
        """

        if type(self.dbc_dirs) == dict:
            self.dbc_files_dict = self.dbc_dirs
            print('\n'.join(self.dbc_dirs.keys()))
        else:
            if os.path.isfile(self.blf_dirs):
                if self.dbc_dirs.endswith('.dbc'):
                    self.dbc_files_dict[os.path.split(self.dbc_dirs)[-1]] = self.dbc_dirs
            else:  # is a folder
                for dbc_dir in os.listdir(self.dbc_dirs):
                    if not dbc_dir.endswith('.dbc'):
                        continue
                    key = os.path.abspath(dbc_dir).split(os.sep)[-1]  # 读取到的路径居然跟系统不一致
                    print(key)
                    self.dbc_files_dict[key] = os.path.join(self.dbc_dirs, dbc_dir)

        self.get_sigs_from_dbc()

    def add_blf(self):
        """
        get blf file path
        :return:
        """

        print('需要解析的文件有： ')
        if type(self.blf_dirs) == dict:
            self.blf_files_dict = self.blf_dirs
            print('\n'.join(self.blf_dirs.keys()))
        else:
            if os.path.isfile(self.blf_dirs):  # is a file
                end_format = self.blf_dirs.split('.')[-1]
                if end_format in ('blf', 'asc'):
                    self.blf_files_dict[os.path.split(self.blf_dirs)[-1]] = self.blf_dirs
                else:
                    self.output_info += '仅支持解析blf和asc\n'
            else:  # is a folder
                for blf_dir in os.listdir(self.blf_dirs):
                    end_format = blf_dir.split('.')[-1]
                    if end_format not in ('blf', 'asc'):
                        continue
                    key = os.path.abspath(blf_dir).split(os.sep)[-1]  # 读取到的路径居然跟系统不一致
                    print(key)
                    self.blf_files_dict[key] = os.path.join(self.blf_dirs, blf_dir)

    @staticmethod
    def collect_choices_from_dbc(dbc_files):
        choices_matrix = {}
        for dbc_file in dbc_files:
            dbc_instance = cantools.db.load_file(dbc_file)
            for msg in dbc_instance.messages:
                for sig in msg.signals:
                    choices_matrix.update({sig.name: sig.choices})
        return choices_matrix

    def get_sigs_from_dbc(self):
        """
        get all signals in dbc
        :return: self.signals_dic signals' dict
        """

        def get_signal_list():
            signal_dic = {}
            for num, dbc in enumerate(self.dbc_files_dict.values()):
                dbc_content = cantools.db.load_file(dbc)
                for msg in dbc_content.messages:
                    for signal in msg.signal_tree:
                        # signal_dic[signal] = (num + 1, hex(msg.frame_id)[2:])
                        signal_dic[signal] = (5, hex(msg.frame_id)[2:])

            self.signals_dic = signal_dic

        get_signal_list()
        # thread = threading.Thread(target=get_signal_list)
        # thread.start()

    def extract_blf(self):
        """
        输出dbc内所有信号
        :return:
        """

        dbc_files = self.dbc_files_dict.values()
        blf_files = self.blf_files_dict.values()

        if not dbc_files or not blf_files:
            # self.status_bar.setText('请先选择DBC和BLF！')
            print('请先选择DBC和BLF！')
            return

        sigs_list = []
        print('需要解析的信号共有： %d' % len(self.signals_dic.keys()))
        for sig_name in self.signals_dic.keys():
            sigs_list.append([self.signals_dic[sig_name][0], self.signals_dic[sig_name][1], sig_name])

        # thread = threading.Thread(target=self.check_sigs_list, args=(dbc_files, blf_files, sigs_list))
        # thread.start()
        print('解析中，请稍等... ')
        signals_value_dict, signals_value_db_dict = self.check_sigs_list(dbc_files, blf_files, sigs_list)

        print('保存数据中...')
        empty_json_path = []
        for blf_file in signals_value_dict.keys():
            this_blf_signal_value = signals_value_dict[blf_file]

            mat_dir = os.path.split(blf_file)
            mat_path = mat_dir[0] + '/' + mat_dir[1][:-4] + '_origin.mat'
            savemat(mat_path, this_blf_signal_value, do_compression=True)

            print('已保存成: %s' % mat_path)
            self.output_info += '解析文件已保存成: %s\n' % mat_path

            none_value_signals = []
            print('记录为空的信号...')
            for signal in this_blf_signal_value.keys():
                if not this_blf_signal_value[signal]:
                    none_value_signals.append(signal)
            json_path = mat_dir[0] + '/' + mat_dir[1][:-4] + '_notValue.json'
            with open(json_path, 'w') as f:
                f.write(json.dumps(none_value_signals, indent=2, ensure_ascii=False))
                f.close()
            print('空值信号已保存成: %s' % json_path)
            empty_json_path.append(json_path)

        self.output_info += '空值信号已保存成: \n'
        self.output_info += '\n'.join(empty_json_path)

        return signals_value_db_dict

    def check_sigs_list(self, dbc_files, blf_files, sigs_list):
        """

        :param dbc_files: dbc所在的文件夹
        :param blf_files: blf所在的文件夹
        :param sigs_list: 需要解析的信号,格式[文件夹中第几个dbc, 信号ID, 信号名]： [[1, 311, 'xxx'],[1, 311, 'xxx'],...]
        :return: signals_value_dict (其中时间为相对时间)
        """
        global ch
        time_start = time.time()
        signals_value_dict = {}
        signals_value_by_time_dict = {}
        if len(sigs_list) < 1:
            print("no sig founded in sigs_list")
            return signals_value_dict, signals_value_by_time_dict
        elif len(dbc_files) < 1:
            print("no dbc_file founded")
            return signals_value_dict, signals_value_by_time_dict
        elif len(blf_files) < 1:
            print("no blf_file founded")
            return signals_value_dict, signals_value_by_time_dict
        # print(sigs_list)
        sigs_list_np = np.array(sigs_list)
        ch_msg_sig_uniq_list = np.unique(sigs_list_np[:, :3], axis=0)
        ch_msg_uniq_list = np.unique(sigs_list_np[:, :2], axis=0)
        ch_dbc = {}
        msg_dbc = defaultdict(list)
        sig_dbc = defaultdict(list)
        sigs_result_dict = defaultdict(list)
        sigs_result_by_time_dict = {}
        sigs_req_in_ch_msg = {}
        msgs_req_in_ch = defaultdict(list)
        for ch, dbc in enumerate(dbc_files):
            ch = ch + 1
            ch_dbc[ch] = cantools.db.load_file(dbc)
            for msg in ch_dbc[ch].messages:
                msg_dbc[ch].append(hex(msg.frame_id))
                sigs_req_in_ch_msg[(ch, hex(msg.frame_id))] = []
                sig_dbc[(ch, hex(msg.frame_id))] = ch_dbc[ch].get_message_by_frame_id(msg.frame_id).signal_tree
        for ch_msg in ch_msg_uniq_list:
            try:
                ch = int(ch_msg[0])
                msg = int(ch_msg[1], 16)
                if hex(msg) in msg_dbc[ch]:
                    msgs_req_in_ch[ch].append(hex(msg))
            except KeyError:
                print("error")

        for ch_msg_sig in ch_msg_sig_uniq_list:
            try:
                ch = int(ch_msg_sig[0])
                msg = int(ch_msg_sig[1], 16)
                sig = str(ch_msg_sig[2])
                if sig in sig_dbc[(ch, hex(msg))]:
                    # print(ch_msg_sig)
                    # print(ch_dbc[ch].get_message_by_frame_id(msg).get_signal_by_name(sig))
                    sigs_req_in_ch_msg[(ch, hex(msg))].append(sig)
                    sigs_result_dict[(ch, hex(msg), sig)] = []  # 用以生成mat文件
            except KeyError:
                print("error")

        # print(time.time())
        for blf_file in blf_files:
            this_blf_signals_value_dict = {}
            end_format = blf_file.split('.')[-1]
            if end_format == 'blf':
                with can.BLFReader(blf_file) as file:
                    now_file = list(file)
                    msgs_req = list(
                        filter(lambda msg_x: hex(msg_x.arbitration_id) in msgs_req_in_ch[msg_x.channel + 1], now_file))
                    this_blf_started_time = now_file[0].timestamp
                    now_file = None
                    print('共有 %d 条数据, 约用时 %d 分钟！' % (len(msgs_req), len(msgs_req) * 0.001082 / 60))
                    self.output_info += '共有 %d 条数据\n' % len(msgs_req)
                    count = 0
                    for msg in msgs_req:
                        try:
                            sigs_result_by_time_dict[str(msg.timestamp) + '_' + hex(msg.arbitration_id)] = dict(
                                {'canID': (ch, hex(msg.arbitration_id)), 'data': {}})
                            # try:
                            msg_content = ch_dbc[msg.channel + 1].decode_message(frame_id_or_name=msg.arbitration_id,
                                                                                 data=msg.data,
                                                                                 decode_choices=False)  # 这里控制数字或字符串
                            for sig_req in sigs_req_in_ch_msg[(msg.channel + 1, hex(msg.arbitration_id))]:
                                sig_t_value = (msg.timestamp, msg_content[sig_req])
                                sigs_result_dict[(msg.channel + 1, hex(msg.arbitration_id), sig_req)].append(sig_t_value)
                                # output sigs_result_dict into csv

                                sigs_result_by_time_dict[str(msg.timestamp) + '_' + hex(msg.arbitration_id)]["data"][
                                    sig_req] = msg_content[sig_req]
                        except Exception as e:
                            count += 1
                            if '无法解析的信号有： ' not in self.output_info:
                                self.output_info += '无法解析的信号有： \n'
                            if hex(msg.arbitration_id) not in self.output_info:
                                self.output_info += hex(msg.arbitration_id)
                                self.output_info += '  '
                                print(e)
                                print(hex(msg.arbitration_id))
                        # except Exception as e:
                        #     print(e)
                        #     print('Sth wrong of canID %s in %f' %(hex(msg.arbitration_id), msg.timestamp))

                    msgs_req = None
            elif end_format == 'asc':
                with can.ASCReader(blf_file) as file:
                    now_file = list(file)
                    msgs_req = list(
                        filter(lambda msg_x: hex(msg_x.arbitration_id) in msgs_req_in_ch[msg_x.channel + 1],
                               now_file))
                    this_blf_started_time = now_file[0].timestamp
                    now_file = None
                    print('共有 %d 条数据, 约用时 %f 分钟！' % (len(msgs_req), len(msgs_req) * 0.001082 / 60))
                    self.output_info += '共有 %d 条数据\n' % len(msgs_req)
                    for msg in msgs_req:
                        try:
                            sigs_result_by_time_dict[str(msg.timestamp) + '_' + hex(msg.arbitration_id)] = dict(
                                {'canID': (ch, hex(msg.arbitration_id)), 'data': {}})
                            msg_content = ch_dbc[msg.channel + 1].decode_message(frame_id_or_name=msg.arbitration_id,
                                                                                 data=msg.data,
                                                                                 decode_choices=False)  # 这里控制数字或字符串
                            for sig_req in sigs_req_in_ch_msg[(msg.channel + 1, hex(msg.arbitration_id))]:
                                sig_t_value = (msg.timestamp, msg_content[sig_req])
                                sigs_result_dict[(msg.channel + 1, hex(msg.arbitration_id), sig_req)].append(
                                    sig_t_value)

                                sigs_result_by_time_dict[str(msg.timestamp) + '_' + hex(msg.arbitration_id)]["data"][
                                    sig_req] = msg_content[sig_req]
                        except Exception as e:
                            print(e)
                            if '无法解析的信号有： ' not in self.output_info:
                                self.output_info += '无法解析的信号有： \n'
                            if hex(msg.arbitration_id) not in self.output_info:
                                self.output_info += hex(msg.arbitration_id)
                                self.output_info += '  '
                        # print('time: %f  canID: %s  dict length:%d' % (msg.timestamp, str(msg.arbitration_id), len(sigs_result_by_time_dict.keys())))
                        # print()

                    msgs_req = None
            else:
                print('暂不支持此数据类型： %s' % blf_file)
                self.output_info += '暂不支持此数据类型： %s\n' % blf_file
                continue

            for key in sigs_result_dict.keys():
                time_value = []
                for i in range(len(sigs_result_dict.get(key))):
                    time_value.append(
                        [sigs_result_dict.get(key)[i][0] - this_blf_started_time, sigs_result_dict.get(key)[i][1]])
                    # time_value.append([sigs_result_dict.get(key)[i][0], sigs_result_dict.get(key)[i][1]])
                this_blf_signals_value_dict[key[2]] = time_value
            signals_value_dict[blf_file] = this_blf_signals_value_dict
            signals_value_by_time_dict[blf_file] = sigs_result_by_time_dict

        print('数据加载完成，耗时%.1f秒' % (time.time() - time_start))

        return signals_value_dict, signals_value_by_time_dict

    def main(self):
        self.add_dbc()
        self.add_blf()

        signals_value_db_dict = self.extract_blf()
        return signals_value_db_dict, self.output_info


if __name__ == '__main__':
    # blf_dirs = r'D:/Files/FX11/data'
    # dbc_dirs = r'D:\Files\FX11\dbc\SDB22200_FX11_High_SafetyCANFD5_220728.dbc'

    blf_dirs = r'D:\Downloads\BLF (1)\BLF'
    dbc_dirs = r'D:\Downloads\dbc'

    loader = canLoader(blf_dirs, dbc_dirs)
    loader.main()

    print('haha')
