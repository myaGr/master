#include <stdio.h>
#include <string>
#include <time.h>

#include "loc_parse.h"
#include "OptionParser.h"
#include "ag_log_parse.h"
#include "ipc_parse.h"
#include "cmn_utils.h"
#include "loc_core_api.h"
#include "mw_logcmprs.h"
#ifdef _WIN32
#include <io.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

using namespace std;

#define READ_BUFFER_SIZE (64 * 1024)

typedef struct {
  char  input_log_filepath[256];
  FILE* input_log_fp;
  char  output_log_filepath[256];
  FILE* output_log_fp;
  uint8_t u_disable_text_parse;
  uint8_t u_disable_ipc_parse;
} LogParserCtrl_t;

static LogParserCtrl_t g_logParserCtrl;
static log_decoder_t gz_log_decoder[2] = { 0 };

uint64_t loc_get_file_size(const char *filename)
{
#ifdef _WIN32
  struct _stat64 file_stat;
  if (_stat64(filename, &file_stat) == 0)
  {
    return file_stat.st_size;
  }
#else
  struct stat64 file_stat;
  if (stat64(filename, &file_stat) == 0)
  {
    return file_stat.st_size;
  }
#endif
  printf("get file size error %s\n", filename);
  return 0;
}

void loc_parse_release(LogParserCtrl_t* gz_logParserCtrl)
{
  if (gz_logParserCtrl->input_log_fp)
  {
    fclose(gz_logParserCtrl->input_log_fp);
  }
  if (gz_logParserCtrl->output_log_fp)
  {
    fclose(gz_logParserCtrl->output_log_fp);
  }
}


static void pb_parsed_data_handler(int8_t decode_type, uint8_t* p_data, uint32_t q_length)
{
  uint32_t q_parse_str_length = 0;
  char parse_str_buffer[5 * 1024] = { 0 };

  FILE* fp = g_logParserCtrl.output_log_fp;

  if (NULL == fp)
  {
    return;
  }

  if ((LOG_TYPE_IPC == decode_type) || (LOG_TYPE_CRC_IPC == decode_type))
  {
    ipc_t z_ipc = { 0 };
    memcpy(&z_ipc, p_data, sizeof(ipc_t));
    if (q_length > sizeof(ipc_t))
    {
      z_ipc.p_data = p_data + sizeof(ipc_t);
    }

    q_parse_str_length += ipc_parse_execute(&z_ipc, parse_str_buffer + q_parse_str_length);
    fwrite(parse_str_buffer, q_parse_str_length, 1, fp);
  }
  else if ((LOG_TYPE_TEXT == decode_type) || (LOG_TYPE_CRC_TEXT == decode_type))
  {
    fwrite(p_data, q_length, 1, fp);
  }
  else if ((LOG_TYPE_PACKAGE == decode_type) || (LOG_TYPE_CRC_PACKAGE == decode_type))
  {
    q_parse_str_length += package_parse_execute(p_data, q_length,
      parse_str_buffer + q_parse_str_length);
    fwrite(parse_str_buffer, q_parse_str_length, 1, fp);
  }
}

void start_parse_log()
{
  uint16_t w_type_id = 0;
  uint64_t q_FileSize = 0;
  uint8_t* input_read_buf = (uint8_t*)malloc(READ_BUFFER_SIZE);
  uint64_t t_input_total_len = 0;

  if (NULL == g_logParserCtrl.input_log_fp)
  {
    printf("Load File fail, exit...\n");
    return;
  }

  q_FileSize = loc_get_file_size(g_logParserCtrl.input_log_filepath);
  printf("Load File Size          :  %0.1f KB\n", q_FileSize / 1024.0);

  char progressBar[100];
  const char* label = "|/-\\";
  memset(progressBar, '\0', sizeof(progressBar));
  progressBar[0] = '=';
  clock_t clockBegin = clock();

  loc_api_MemoryRegister_t z_MemoryRegister = { 0 };
  z_MemoryRegister.u_type = LOC_API_MEMORY_OS;
  z_MemoryRegister.pool_addr[0] = NULL;
  z_MemoryRegister.pool_size[0] = 0;
  z_MemoryRegister.pool_count = 1;
  z_MemoryRegister.alloc = NULL;
  z_MemoryRegister.free = NULL;
  loc_api_Register_MemoryPool(&z_MemoryRegister);

  while (!feof(g_logParserCtrl.input_log_fp))
  {
    size_t buf_len = fread(input_read_buf, 1, READ_BUFFER_SIZE, g_logParserCtrl.input_log_fp);
    if (buf_len < 1)
    {
      break;
    }
    t_input_total_len += buf_len;

    loc_core_log_parse(&gz_log_decoder[0], pb_parsed_data_handler, input_read_buf, buf_len, TRUE);

    
    uint8_t rate = (uint32_t)(100.0 * (t_input_total_len) / q_FileSize);
    if (rate % 2 == 0)
    {
      if (rate == 100)
      {
        
        printf("Complete [%-51s]%3d%%[%0.3fs][%c]\r",
          progressBar, rate, (clock() - clockBegin) / (double)CLOCKS_PER_SEC, label[rate % 4]);
      }
      else
      {
        printf("Runing   [%-51s]%3d%%[%0.3fs][%c]\r",
          progressBar, rate, (clock() - clockBegin) / (double)CLOCKS_PER_SEC, label[rate % 4]);
      }
    }
    else
    {
      printf("Runing   [%-51s]%3d%%[%0.3fs][%c]\r",
        progressBar, rate, (clock() - clockBegin) / (double)CLOCKS_PER_SEC, label[rate % 4]);
    }
    progressBar[rate / 2] = '=';
    fflush(stdout);
   }
  free(input_read_buf);
}

void process_args(optparse::Values& options)
{
  const char* log_file = options["file"].c_str();
  memcpy(g_logParserCtrl.input_log_filepath, log_file, strlen(log_file));

  char drive[64] = { 0 };
  char dir[64] = { 0 };
  char fname[64] = { 0 };
  char ext[64] = { 0 };
  _splitpath(log_file, drive, dir, fname, ext);

  if (options.is_set(string("output")))
  {
    const char* f = options["output"].c_str();
    memset(dir, 0, sizeof(dir));
    memcpy(dir, f, strlen(f));
  }

  sprintf(g_logParserCtrl.output_log_filepath, "%s%s%s%s.txt", drive, dir, "Parse_", fname);

  g_logParserCtrl.input_log_fp = fopen(g_logParserCtrl.input_log_filepath, "rb");
  if (NULL == g_logParserCtrl.input_log_fp)
  {
    printf("[Error]Open File %s Fail, exit...\n", g_logParserCtrl.input_log_filepath);
    return;
  }

  g_logParserCtrl.output_log_fp = fopen(g_logParserCtrl.output_log_filepath, "w+");
  if (NULL == g_logParserCtrl.output_log_fp)
  {
    printf("[Error]Open File %s Fail, exit...\n", g_logParserCtrl.output_log_filepath);
    return;
  }

  g_logParserCtrl.u_disable_text_parse = (uint8_t)(int)options.get("disable_text");
  g_logParserCtrl.u_disable_ipc_parse = (uint8_t)(int)options.get("disable_ipc");

  ipc_parse_config_output_file(drive, dir, (char*)"Parse_", fname);
}

int main(int argc, char* argv[])
{
  char version_str[256] = { 0 };
  get_version(version_str);

  const string description = "Location Engine Log Parser Tool";
  const string usage = "usage: %prog [OPTION]...";
  optparse::OptionParser parser = optparse::OptionParser()
    .description(description)
    .usage(usage)
    .version(version_str);

  parser.add_option("-f", "--file")
    .action("store")
    .type("string")
    .set_default("log.bin")
    .help("playback file default:%default");

  parser.add_option("-o", "--output")
    .action("store")
    .type("string")
    .help("output dir");

  parser.add_option("--disable_text")
    .action("count")
    .help("Disable text log parse");

  parser.add_option("--disable_ipc")
    .action("count")
    .help("Disable ipc log parse");

  optparse::Values options = parser.parse_args(argc, argv);
  parser.print_help();
  parser.print_version();
  parser.print_args();

  process_args(options);
  start_parse_log();
  ipc_parse_release();
  loc_parse_release(&g_logParserCtrl);
  return 0;
}
