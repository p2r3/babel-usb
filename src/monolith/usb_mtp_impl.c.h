/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Ennebi Elettronica (https://ennebielettronica.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <sys/dirent.h>
#include <sys/errno.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <math.h>
#include "esp_log.h"
#include "tusb.h"
#include "util.h"
#include "utf8-utf16-converter.h"

#define TU_FIELD_SIZE(_type, _field)  (sizeof(((_type *)0)->_field))

#ifdef MTP_ESP_LOG
#undef MTP_ESP_LOG
#endif
#define MTP_ESP_LOG ESP_LOGD

//--------------------------------------------------------------------+
// Dataset
//--------------------------------------------------------------------+

//------------- device info -------------//
#define DEV_INFO_MANUFACTURER   "p2r3"
#define DEV_INFO_MODEL          "USB of Babel"
#define DEV_INFO_VERSION        "1.0"
#define DEV_PROP_FRIENDLY_NAME  "USB of Babel"

//------------- storage info -------------//
#define STORAGE_DESCRIPTION { 'd', 'i', 's', 'k', 0 }
#define VOLUME_IDENTIFIER { 'v', 'o', 'l', 0 }

typedef MTP_STORAGE_INFO_STRUCT(TU_ARRAY_SIZE((uint16_t[]) STORAGE_DESCRIPTION),
                                TU_ARRAY_SIZE(((uint16_t[])VOLUME_IDENTIFIER))
) storage_info_t;

#define CFG_EXAMPLE_MTP_READONLY

storage_info_t storage_info = {
  #ifdef CFG_EXAMPLE_MTP_READONLY
  .storage_type = MTP_STORAGE_TYPE_FIXED_ROM,
  #else
  .storage_type = MTP_STORAGE_TYPE_FIXED_RAM,
  #endif

  .filesystem_type = MTP_FILESYSTEM_TYPE_GENERIC_HIERARCHICAL,
  .access_capability = MTP_ACCESS_CAPABILITY_READ_ONLY_WITHOUT_OBJECT_DELETION,
  .max_capacity_in_bytes = 0, // calculated at runtime
  .free_space_in_bytes = 0, // calculated at runtime
  .free_space_in_objects = 0, // calculated at runtime
  .storage_description = {
    .count = (TU_FIELD_SIZE(storage_info_t, storage_description)-1) / sizeof(uint16_t),
    .utf16 = STORAGE_DESCRIPTION
  },
  .volume_identifier = {
    .count = (TU_FIELD_SIZE(storage_info_t, volume_identifier)-1) / sizeof(uint16_t),
    .utf16 = VOLUME_IDENTIFIER
  }
};

//--------------------------------------------------------------------+
// MTP FILESYSTEM
//--------------------------------------------------------------------+
// only allow to add 1 more object to make it simpler to manage memory
#define FS_MAX_FILE_COUNT 3UL
#define FS_MAX_FILENAME_LEN 16

#ifdef CFG_EXAMPLE_MTP_READONLY
  #define FS_MAX_CAPACITY_BYTES  0
#else
  #define FS_MAX_CAPACITY_BYTES (4 * 1024UL)

  // object data buffer (excluding 2 predefined files) with simple allocation pointer
  uint8_t fs_buf[FS_MAX_CAPACITY_BYTES];
#endif

#define FS_FIXED_DATETIME "20250808T173500.0" // "YYYYMMDDTHHMMSS.s"

// vvv My LittleFS logic
#define MTP_FILENAME_LENGTH 63
// #define MTP_HANDLE_TABLE_SIZE 32
typedef uint32_t fs_handle_t;
#define FS_INVALID_HANDLE ((fs_handle_t)UINT_MAX)

// typedef struct {
//   fs_handle_t handle;             // Handle assigned to this entry.
//   fs_handle_t parent_handle;      // When all bits are set, the parent is root directory
//   bool is_dir;
//   char name[MTP_FILENAME_LENGTH]; // When first character is 0x00, the entry is empty
// } fs_handletable_entry_t;

// Because MTP requires the responder to provide a consistent handle that can be used in any later
// time of a session to manipulate a file, for convenience we just use this mega handle table to
// record all the files we have in the filesystem. It's redundant, (because filesystem should have
// done the bookkeeping!) but because in LittleFS we don't have a global identifier like inodes,
// it's inherently hard for responder to keep all the handle info with a filesystem that can have
// many more files than available RAM. Therefore, our total object count will pretty much be limited
// to the size of this handle table. But considering how many files you can actually fit onto the
// device, 32 objects should be more than enough already.
//
// In this example, we're implementing a simple filesystem with only one layer of directory. The
// reason is to deliberately limit the complexity.
// typedef struct {
//   fs_handletable_entry_t handles[MTP_HANDLE_TABLE_SIZE];

//   uint32_t handles_used;
// } fs_handletable;
// static fs_handletable handle_table;
FILE *current_file = NULL;
fs_handle_t current_handle = FS_INVALID_HANDLE;
size_t current_file_size = 0;
// ^^^ My LittleFS logic

enum {
  SUPPORTED_STORAGE_ID = 0x00010001u // physical = 1, logical = 1
};

static int32_t fs_get_device_info(tud_mtp_cb_data_t* cb_data);
static int32_t fs_open_close_session(tud_mtp_cb_data_t* cb_data);
static int32_t fs_get_storage_ids(tud_mtp_cb_data_t* cb_data);
static int32_t fs_get_storage_info(tud_mtp_cb_data_t* cb_data);
static int32_t fs_get_device_properties(tud_mtp_cb_data_t* cb_data);
static int32_t fs_get_object_handles(tud_mtp_cb_data_t* cb_data);
static int32_t fs_get_object_info(tud_mtp_cb_data_t* cb_data);
static int32_t fs_get_object(tud_mtp_cb_data_t* cb_data);
static int32_t fs_delete_object(tud_mtp_cb_data_t* cb_data);
static int32_t fs_send_object_info(tud_mtp_cb_data_t* cb_data);
static int32_t fs_send_object(tud_mtp_cb_data_t* cb_data);
static int32_t fs_format_store(tud_mtp_cb_data_t* cb_data);

typedef int32_t (*fs_op_handler_t)(tud_mtp_cb_data_t* cb_data);
typedef struct {
  uint32_t op_code;
  fs_op_handler_t handler;
}fs_op_handler_dict_t;

fs_op_handler_dict_t fs_op_handler_dict[] = {
  { MTP_OP_GET_DEVICE_INFO,       fs_get_device_info    },
  { MTP_OP_OPEN_SESSION,          fs_open_close_session }, // init
  { MTP_OP_CLOSE_SESSION,         fs_open_close_session },
  { MTP_OP_GET_STORAGE_IDS,       fs_get_storage_ids       },
  { MTP_OP_GET_STORAGE_INFO,      fs_get_storage_info      },
  { MTP_OP_GET_DEVICE_PROP_DESC,  fs_get_device_properties  },
  { MTP_OP_GET_DEVICE_PROP_VALUE, fs_get_device_properties },
  { MTP_OP_GET_OBJECT_HANDLES,    fs_get_object_handles    }, // ls
  { MTP_OP_GET_OBJECT_INFO,       fs_get_object_info       }, // stat
  { MTP_OP_GET_OBJECT,            fs_get_object            }, // read file
  { MTP_OP_DELETE_OBJECT,         fs_delete_object         },
  { MTP_OP_SEND_OBJECT_INFO,      fs_send_object_info      },
  { MTP_OP_SEND_OBJECT,           fs_send_object           },
  { MTP_OP_FORMAT_STORE,          fs_format_store          },
};

static bool is_session_opened = false;
// static uint32_t send_obj_handle = 0;

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

// static fs_handle_t handle_self_inc = 0;
// static fs_handle_t fs_assign_new_handle(void)
// {
//   // Use incrementing handle for new objects in handle table. MTP does not allow reusing handles of
//   // deleted objects.
//   // When the session is terminated, we can reset it to 0.
//   return ++handle_self_inc;
// }

// static fs_handletable_entry_t *fs_get_handle_entry(fs_handletable *handle_table, fs_handle_t handle)
// {
//   for (int ii = 0; ii < MTP_HANDLE_TABLE_SIZE; ii++) {
//     if (handle_table->handles[ii].handle == handle) {
//       return &handle_table->handles[ii];
//     }
//   }
//   return NULL;
// }

// static void fs_handletable_regenerate(fs_handletable *handle_table) {

//   int curr_handle = fs_assign_new_handle();
//   strncpy(handle_table->handles[curr_handle].name, "dir1", MTP_FILENAME_LENGTH);
//   handle_table->handles[curr_handle].parent_handle = 0;
//   handle_table->handles[curr_handle].handle = curr_handle;
//   handle_table->handles[curr_handle].is_dir = true;

//   curr_handle = fs_assign_new_handle();
//   strncpy(handle_table->handles[curr_handle].name, "dir2", MTP_FILENAME_LENGTH);
//   handle_table->handles[curr_handle].parent_handle = 0;
//   handle_table->handles[curr_handle].handle = curr_handle;
//   handle_table->handles[curr_handle].is_dir = true;

//   curr_handle = fs_assign_new_handle();
//   strncpy(handle_table->handles[curr_handle].name, "file1", MTP_FILENAME_LENGTH);
//   handle_table->handles[curr_handle].parent_handle = 0;
//   handle_table->handles[curr_handle].handle = curr_handle;

//   curr_handle = fs_assign_new_handle();
//   strncpy(handle_table->handles[curr_handle].name, "file2", MTP_FILENAME_LENGTH);
//   handle_table->handles[curr_handle].parent_handle = 0;
//   handle_table->handles[curr_handle].handle = curr_handle;

//   handle_table->handles_used = curr_handle;

  //   int ii = fs_assign_new_handle();
  //   auto root = opendir("/littlefs");
  //   char path_buf[200];
  //   struct dirent *rootitem;
  //   if (root == NULL) {
  //     ESP_LOGE("MtpInit", "Cannot opendir(\"/littlefs\"), got NULL");
  //     return;
  //   }
  //   while ((rootitem = readdir(root)) != NULL) {
  //     // One root item was found. Record it in the handle table
  //     strncpy(handle_table->handles[ii].name, rootitem->d_name, MTP_FILENAME_LENGTH);
  //     handle_table->handles[ii].parent_handle = 0;
  //     handle_table->handles[ii].handle = ii;
  //     MTP_ESP_LOG("MtpInit", "Handle %d = /%s", ii, rootitem->d_name);
  //     if ((ii = fs_assign_new_handle()) >= MTP_HANDLE_TABLE_SIZE) {
  //       ESP_LOGW("MtpInit", "Handle table full, stopping handle table init");
  //       goto cleanup;
  //     }

  //     // If it is a directory, we look inside too
  //     if (rootitem->d_type == DT_DIR) {
  //       auto parent_handle = ii;
  //       handle_table->handles[parent_handle].is_dir = true;

  //       strcpy(path_buf, "/littlefs/");
  //       strcat(path_buf, rootitem->d_name);
  //       auto subdir = opendir(path_buf);
  //       if (subdir == NULL) {
  //         ESP_LOGE("MtpInit", "Cannot opendir(\"/littlefs/%s\"), got NULL", rootitem->d_name);
  //         continue;
  //       }
  //       struct dirent *subdiritem;
  //       while ((subdiritem = readdir(subdir)) != NULL) {
  //         // One subdir item was found. Record it in the handle
  //         strncpy(handle_table->handles[ii].name, subdiritem->d_name, MTP_FILENAME_LENGTH);
  //         handle_table->handles[ii].parent_handle = parent_handle;
  //         handle_table->handles[ii].is_dir = subdiritem->d_type == DT_DIR;
  //         MTP_ESP_LOG("MtpInit", "Handle %d = /%s/%s", ii, rootitem->d_name, subdiritem->d_name);
  //         if ((ii = fs_assign_new_handle()) >= MTP_HANDLE_TABLE_SIZE) {
  //           ESP_LOGW("MtpInit", "Handle table full, stopping handle table init");
  //           goto cleanup;
  //         }
  //       }
  //       closedir(subdir);
  //     }
  //   }
  // cleanup:
  //   closedir(root);
  //   handle_table->handles_used = ii;
// }

// static fs_handle_t fs_handletable_find_empty_entry(const fs_handletable *handle_table) {
//   for (int ii = 0; ii < MTP_HANDLE_TABLE_SIZE; ii++) {
//     if (handle_table->handles[ii].name[0] == '\0') {
//       MTP_ESP_LOG("MtpFS", "Found empty handle slot %d", ii);
//       return ii;
//     }
//   }
//   return FS_INVALID_HANDLE;
// }

// static bool fs_handle_valid(fs_handletable *handle_table, fs_handle_t handle) {
//   for (int ii = 0; ii < MTP_HANDLE_TABLE_SIZE; ++ii) {
//     fs_handletable_entry_t *entry = &handle_table->handles[ii];
//     if (entry->handle == handle && entry->name[0] != '\0') {
//       return true;
//     }
//   }
//   return false;
// }

// static bool fs_path_from_handle(fs_handletable *handle_table, fs_handle_t handle, char *path_out, int buf_len)
// {
//   if (!fs_handle_valid(handle_table, handle)) {
//     return false;
//   }
//   fs_handletable_entry_t *entry = fs_get_handle_entry(handle_table, handle);
//   strlcpy(path_out, "/littlefs/", buf_len);
//   if (entry->parent_handle != 0) {
//     fs_handletable_entry_t *parent_entry = fs_get_handle_entry(handle_table, entry->parent_handle);
//     strlcat(path_out, parent_entry->name, buf_len);
//     strlcat(path_out, "/", buf_len);
//   }
//   strlcat(path_out, entry->name, buf_len);
//   MTP_ESP_LOG("MtpFS", "Mapped Handle %d to File %s", handle, path_out);
//   return true;
// }

// static int fs_path_create(fs_handletable *handle_table, fs_handle_t parent_handle, const char *name, char *path_out, int buf_len)
// {
//   if (parent_handle != 0) {
//     fs_handletable_entry_t *parent_entry = fs_get_handle_entry(handle_table, parent_handle);
//     if (parent_entry == NULL) {
//       return -ENOENT;
//     }
//     strlcpy(path_out, "/littlefs/", buf_len);
//     strlcat(path_out, parent_entry->name, buf_len);
//     strlcat(path_out, "/", buf_len);
//   } else {
//     strlcpy(path_out, "/littlefs/", buf_len);
//   }
//   strlcat(path_out, name, buf_len);
//   return 0;
// }

// static int fs_stat_handle(fs_handletable *handle_table, fs_handle_t handle, struct stat *stat_buf, fs_handletable_entry_t **handletable_entry)
// {
//   char path_buf[200];
//   if (!fs_path_from_handle(handle_table, handle, path_buf, sizeof(path_buf))) {
//     return -EINVAL;
//   }
//   *handletable_entry = fs_get_handle_entry(handle_table, handle);
//   stat_buf->st_size = 1024;
//   stat_buf->st_mode = (*handletable_entry)->is_dir ? (S_IFDIR | 0755) : (S_IFREG | 0644);
//   return 0;
//   // return stat(path_buf, stat_buf);
// }

// static FILE *fs_open_handle(fs_handletable *handle_table, fs_handle_t handle, const char* restrict mode)
// {
//   char path_buf[200];
//   if (current_file != NULL && current_handle == handle) {
//     return current_file;
//   }
//   if (!fs_path_from_handle(handle_table, handle, path_buf, sizeof(path_buf))) {
//     return NULL;
//   }
//   current_handle = handle;
//   current_file = fopen(path_buf, mode);

//   // Resolve file size when in read mode
//   if (mode[0] == 'r') {
//     fseek(current_file, 0, SEEK_END);
//     current_file_size = ftell(current_file);
//     fseek(current_file, 0, SEEK_SET);
//   }

//   return current_file;
// }

// static bool fs_can_create_file(fs_handletable *handle_table, size_t size)
// {
//   size_t capacity_bytes, used_bytes;
//   if (handle_table->handles_used == MTP_HANDLE_TABLE_SIZE) {
//     return false;
//   }
//   esp_littlefs_info("littlefs", &capacity_bytes, &used_bytes);
//   if (capacity_bytes - used_bytes < size) {
//     return false;
//   }
//   return true;
//   return false;
// }

// static fs_handle_t fs_create_file(fs_handletable *handle_table, fs_handle_t parent_handle, const char *name)
// {
//   char pathbuf[200];
//   int retval = fs_path_create(handle_table, parent_handle, name, pathbuf, sizeof(pathbuf));
//   if (retval != 0) {
//     ESP_LOGE("MtpFS", "fs_create_file failed to generate path: %d", retval);
//     return FS_INVALID_HANDLE;
//   }

//   int handle_slot = fs_handletable_find_empty_entry(handle_table);
//   if (handle_slot == FS_INVALID_HANDLE) {
//     ESP_LOGE("MtpFS", "fs_create_file failed to find available entry in handle table");
//     return handle_slot;
//   }

//   fs_handle_t handle = fs_assign_new_handle();

//   current_file = fopen(pathbuf, "w");
//   if (current_file == NULL) {
//     ESP_LOGE("MtpFS", "fs_create_file failed to open file in write mode: %s", pathbuf);
//     return FS_INVALID_HANDLE;
//   }

//   fs_handletable_entry_t *entry = &handle_table->handles[handle_slot];
//   entry->parent_handle = parent_handle;
//   entry->handle = handle;
//   strlcpy(entry->name, name, MTP_FILENAME_LENGTH);

//   current_handle = handle;
//   handle_table->handles_used++;
//   MTP_ESP_LOG("MtpFS", "Created file for write, handle=%d, path=%s", handle, pathbuf);
//   return handle;
// }

// static void fs_close_handle(fs_handle_t handle, FILE *file)
// {
//   if (current_file != file || current_handle != handle) {
//     ESP_LOGE("MtpFS", "fs_close_handle check fail: mismatched state");
//     return;
//   }

//   fclose(file);
//   current_handle = FS_INVALID_HANDLE;
//   current_file = NULL;
// }

// static int fs_delete_handle(fs_handletable *handle_table, fs_handle_t handle)
// {
//   for (int ii = 0; ii < MTP_HANDLE_TABLE_SIZE; ii++) {
//     fs_handletable_entry_t *entry = &handle_table->handles[ii];
//     if (entry->handle == handle && entry->name[0] != '\0') {
//       entry->name[0] = '\0';
//       handle_table->handles_used--;
//       return 0;
//     }
//   }
//   return -ENOENT;
// }

//--------------------------------------------------------------------+
// Control Request callback
//--------------------------------------------------------------------+
bool tud_mtp_request_cancel_cb(tud_mtp_request_cb_data_t* cb_data) {
  mtp_request_reset_cancel_data_t cancel_data;
  memcpy(&cancel_data, cb_data->buf, sizeof(cancel_data));
  (void) cancel_data.code;
  (void ) cancel_data.transaction_id;
  // Dump the file currently working on.
  // fs_close_handle(current_handle, current_file);
  return true;
}

// Invoked when received Device Reset request
// return false to stall the request
bool tud_mtp_request_device_reset_cb(tud_mtp_request_cb_data_t* cb_data) {
  (void) cb_data;
  return true;
}

// Invoked when received Get Extended Event request. Application fill callback data's buffer for response
// return negative to stall the request
int32_t tud_mtp_request_get_extended_event_cb(tud_mtp_request_cb_data_t* cb_data) {
  (void) cb_data;
  return false; // not implemented yet
}

// Invoked when received Get DeviceStatus request. Application fill callback data's buffer for response
// return negative to stall the request
int32_t tud_mtp_request_get_device_status_cb(tud_mtp_request_cb_data_t* cb_data) {
  uint16_t* buf16 = (uint16_t*)(uintptr_t) cb_data->buf;
  buf16[0] = 4; // length
  buf16[1] = MTP_RESP_OK; // status
  return 4;
}

//--------------------------------------------------------------------+
// Bulk Only Protocol
//--------------------------------------------------------------------+
int32_t tud_mtp_command_received_cb(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;
  fs_op_handler_t handler = NULL;
  for (size_t i = 0; i < TU_ARRAY_SIZE(fs_op_handler_dict); i++) {
    if (fs_op_handler_dict[i].op_code == command->header.code) {
      handler = fs_op_handler_dict[i].handler;
      break;
    }
  }

  int32_t resp_code;
  if (handler == NULL) {
    resp_code = MTP_RESP_OPERATION_NOT_SUPPORTED;
  } else {
    resp_code = handler(cb_data);
    if (resp_code > MTP_RESP_UNDEFINED) {
      // send response if needed
      io_container->header->code = (uint16_t)resp_code;
      tud_mtp_response_send(io_container);
    }
  }

  return resp_code;
}

int32_t tud_mtp_data_xfer_cb(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;

  fs_op_handler_t handler = NULL;
  for (size_t i = 0; i < TU_ARRAY_SIZE(fs_op_handler_dict); i++) {
    if (fs_op_handler_dict[i].op_code == command->header.code) {
      handler = fs_op_handler_dict[i].handler;
      break;
    }
  }

  int32_t resp_code;
  if (handler == NULL) {
    resp_code = MTP_RESP_OPERATION_NOT_SUPPORTED;
  } else {
    resp_code = handler(cb_data);
    if (resp_code > MTP_RESP_UNDEFINED) {
      // send response if needed
      io_container->header->code = (uint16_t)resp_code;
      tud_mtp_response_send(io_container);
    }
  }

  return resp_code;
}

int32_t tud_mtp_data_complete_cb(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* resp = &cb_data->io_container;
  switch (command->header.code) {
    case MTP_OP_SEND_OBJECT_INFO: {
      // fs_handletable_entry_t *entry = fs_get_handle_entry(&handle_table, current_handle);
      // if (entry == NULL) {
      //   resp->header->code = MTP_RESP_INVALID_OBJECT_HANDLE;
      //   break;
      // }
      // parameter is: storage id, parent handle, new handle
      mtp_container_add_uint32(resp, SUPPORTED_STORAGE_ID);
      mtp_container_add_uint32(resp, 0);
      mtp_container_add_uint32(resp, current_handle);
      resp->header->code = MTP_RESP_OK;
      break;
    }

    default:
      resp->header->code = (cb_data->xfer_result == XFER_RESULT_SUCCESS) ? MTP_RESP_OK : MTP_RESP_GENERAL_ERROR;
      break;
  }

  tud_mtp_response_send(resp);
  return 0;
}

int32_t tud_mtp_response_complete_cb(tud_mtp_cb_data_t* cb_data) {
  (void) cb_data;
  return 0; // nothing to do
}

// char babel_log[16384] = {0};
// char babel_log[16384] = "Hello, world\n\0";

//--------------------------------------------------------------------+
// File System Handlers
//--------------------------------------------------------------------+
static int32_t fs_get_device_info(tud_mtp_cb_data_t* cb_data) {
  // Device info is already prepared up to playback formats. Application only need to add string fields
  mtp_container_info_t* io_container = &cb_data->io_container;
  mtp_container_add_cstring(io_container, DEV_INFO_MANUFACTURER);
  mtp_container_add_cstring(io_container, DEV_INFO_MODEL);
  mtp_container_add_cstring(io_container, DEV_INFO_VERSION);

  enum { MAX_SERIAL_NCHARS = 32 };
  uint16_t serial_utf16[MAX_SERIAL_NCHARS+1];
  utilGetMacAddressNoDelimiterUtf16le(serial_utf16);
  serial_utf16[tu_min32(12, MAX_SERIAL_NCHARS)] = 0; // ensure null termination
  mtp_container_add_string(io_container, serial_utf16);

  tud_mtp_data_send(io_container);
  return 0;
}

#define BABEL_MAX_NAME_LENGTH 2
#define BABEL_ALPHABET_LENGTH 70

const char alphabet[BABEL_ALPHABET_LENGTH] = {
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
  'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
  'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
  'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
  'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
  'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
  'w', 'x', 'y', 'z', '0', '1', '2', '3',
  '4', '5', '6', '7', '8', '9', '!', ' ',
  '&', '(', ')', '-', '_', '+', '[', ']',
  '^', '='
};

uint8_t output_file[4096] = {0};
size_t output_file_len = 0;

uint32_t handle_count = 0;
uint32_t path_depth = 0;
uint32_t prev_parent_handle = 0;

static int32_t fs_open_close_session(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  if (command->header.code == MTP_OP_OPEN_SESSION) {
    if (is_session_opened) {
      return MTP_RESP_SESSION_ALREADY_OPEN;
    }
    is_session_opened = true;

    if (!handle_count) {
      // Calculate total amount of objects in each directory
      handle_count = 1;
      for (int i = 0; i < BABEL_MAX_NAME_LENGTH; i++) {
        handle_count *= BABEL_ALPHABET_LENGTH;
      }
      // Add one handle for the sole file in the directory
      handle_count++;
    }

    output_file_len = 0;
    output_file[0] = 0;

    // Upon session open, we regenerate the handle table
    // fs_handletable_regenerate(&handle_table);
  } else { // close session
    if (!is_session_opened) {
      return MTP_RESP_SESSION_NOT_OPEN;
    }
    is_session_opened = false;
    // handle_self_inc = 0;
  }
  return MTP_RESP_OK;
}

static int32_t fs_get_storage_ids(tud_mtp_cb_data_t* cb_data) {
  mtp_container_info_t* io_container = &cb_data->io_container;
  uint32_t storage_ids [] = { SUPPORTED_STORAGE_ID };
  mtp_container_add_auint32(io_container, 1, storage_ids);
  tud_mtp_data_send(io_container);
  return 0;
}

static int32_t fs_get_storage_info(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;
  const uint32_t storage_id = command->params[0];
  TU_VERIFY(SUPPORTED_STORAGE_ID == storage_id, -1);
  // update storage info with current free space
  // size_t capacity_bytes, used_bytes;
  // esp_littlefs_info("littlefs", &capacity_bytes, &used_bytes);
  // storage_info.max_capacity_in_bytes = capacity_bytes;
  // storage_info.free_space_in_objects = MTP_HANDLE_TABLE_SIZE - handle_table.handles_used;
  // storage_info.free_space_in_bytes = capacity_bytes - used_bytes;
  storage_info.max_capacity_in_bytes = ~0 / 2;
  storage_info.free_space_in_objects = 0;
  storage_info.free_space_in_bytes = 0;
  mtp_container_add_raw(io_container, &storage_info, sizeof(storage_info));
  tud_mtp_data_send(io_container);
  return 0;
}

static int32_t fs_get_device_properties(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;
  const uint16_t dev_prop_code = (uint16_t) command->params[0];

  if (command->header.code == MTP_OP_GET_DEVICE_PROP_DESC) {
    // get describing dataset
    mtp_device_prop_desc_header_t device_prop_header;
    device_prop_header.device_property_code = dev_prop_code;
    switch (dev_prop_code) {
      case MTP_DEV_PROP_DEVICE_FRIENDLY_NAME:
        device_prop_header.datatype = MTP_DATA_TYPE_STR;
        device_prop_header.get_set = MTP_MODE_GET;
        mtp_container_add_raw(io_container, &device_prop_header, sizeof(device_prop_header));
        mtp_container_add_cstring(io_container, DEV_PROP_FRIENDLY_NAME); // factory
        mtp_container_add_cstring(io_container, DEV_PROP_FRIENDLY_NAME); // current
        mtp_container_add_uint8(io_container, 0); // no form
        tud_mtp_data_send(io_container);
        break;

      default:
        return MTP_RESP_PARAMETER_NOT_SUPPORTED;
    }
  } else {
    // get value
    switch (dev_prop_code) {
      case MTP_DEV_PROP_DEVICE_FRIENDLY_NAME:
        mtp_container_add_cstring(io_container, DEV_PROP_FRIENDLY_NAME);
        tud_mtp_data_send(io_container);
        break;

      default:
        return MTP_RESP_PARAMETER_NOT_SUPPORTED;
    }
  }
  return 0;
}

void fs_output_add (uint32_t value) {
  uint32_t index = 0;
  uint32_t carry = 0;
  while (value || carry) {
    int digit = (value & 0xFF) + output_file[index] + carry;
    output_file[index] = digit & 0xFF;
    carry = digit >> 8;
    value >>= 8;
    if (++index > output_file_len) {
      output_file[index] = 0;
      output_file_len = index;
    }
  }
}

void fs_output_multiply (uint32_t value) {
  uint32_t index = 0;
  uint32_t carry = 0;
  while (index < output_file_len || carry) {
    uint32_t digit = output_file[index] * value + carry;
    output_file[index] = digit & 0xFF;
    carry = digit >> 8;
    if (++index > output_file_len) {
      output_file[index] = 0;
      output_file_len = index;
    }
  }
}

void fs_output_bytes_dec () {
  for (int i = 0; i < output_file_len - 1; i ++) {
    int j = i;
    while (1) {
      if (output_file[j] == 0) {
        output_file[j] = 255;
      } else {
        output_file[j] --;
        break;
      }
      j ++;
    }
  }
  if (output_file[output_file_len - 1] == 0) {
    output_file_len --;
  } else {
    output_file[output_file_len - 1] --;
  }
}

void fs_output_bytes_inc () {
  for (int i = 0; i < output_file_len; i ++) {
    int j = i;
    while (1) {
      if (output_file[j] == 255) {
        output_file[j] = 0;
      } else {
        output_file[j] ++;
        break;
      }
      if (++j > output_file_len) {
        output_file[j] = 0;
        output_file_len = j;
      }
    }
  }
}

void fs_append_output (uint32_t handle) {

  // Increment every byte of the output file buffer
  /**
   * This converts the arbitrary byte buffer to a valid base-256 number
   * without any leading zeroes, so that it can be operated on with basic
   * arithmetic. Then, once we're done, we decrement every byte to convert
   * it back to a byte buffer that supports leading null bytes.
   */
  fs_output_bytes_inc();

  // Shift buffer up by amount of possible combinations (base)
  fs_output_multiply(handle_count - 1);

  // Add current handle to least significant end of buffer
  fs_output_add(handle);

  // Decrement every byte of the output file buffer, undoing the first operation
  fs_output_bytes_dec();

}

// void fs_reverse_output (uint32_t handle) {

//   // Subtract handle from buffer
//   uint32_t index = 0;
//   uint32_t carry = 0;
//   while (handle || carry) {
//     int value = output_file[index] - (handle & 0xFF) - carry;
//     output_file[index] = value < 0 ? 256 + value : value;
//     carry = value < 0;
//     handle >>= 8;
//     if (++index > output_file_len) {
//       output_file[index] = 0;
//       output_file_len = index;
//     }
//   }

//   // Divide buffer by alphabet length
//   // This should be a clean division with no remainder
//   index = output_file_len;
//   carry = 0;
//   while (index-- > 0) {
//     uint32_t value = output_file[index] + carry * 256;
//     output_file[index] = value / BABEL_ALPHABET_LENGTH;
//     carry = value % BABEL_ALPHABET_LENGTH;
//   }

//   // Trim leading zero bytes
//   while (output_file_len > 0 && output_file[output_file_len - 1] == 0) {
//     output_file_len--;
//   }

// }

static int32_t fs_get_object_handles(tud_mtp_cb_data_t* cb_data) {
  // `ls /<folder_in_question>`
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;

  const uint32_t storage_id = command->params[0];
  const uint32_t obj_format = command->params[1]; // optional
  const uint32_t parent_handle = command->params[2];
  (void)obj_format;

  if (storage_id != 0xFFFFFFFF && storage_id != SUPPORTED_STORAGE_ID) {
    return MTP_RESP_INVALID_STORAGE_ID;
  }

  if (cb_data->phase == MTP_PHASE_COMMAND) {

    if (parent_handle != 0xFFFFFFFF) {
      if (prev_parent_handle != parent_handle) {
        fs_append_output((parent_handle - 1) % handle_count + 1);
        prev_parent_handle = parent_handle;
        path_depth++;
      }
      // snprintf(babel_log + strlen(babel_log), sizeof(babel_log), "get_obj_handles %lu\n", parent_handle);
    } else {
      // snprintf(babel_log + strlen(babel_log), sizeof(babel_log), "get_obj_handles 0\n");
      output_file_len = 0;
      output_file[0] = 0;
      prev_parent_handle = 0;
      path_depth = 0;
    }

    // Payload space in the first packet (after the 12-byte container header)
    const uint32_t first_payload_bytes = io_container->payload_bytes;

    // Write the MTP array count field (4 bytes) into the payload
    uint32_t* p = (uint32_t*) io_container->payload;
    *p++ = handle_count;
    io_container->header->len += 4;

    // Fill the remaining space in the first packet with as many handles as fit
    const uint32_t first_batch = tu_min32(handle_count, (first_payload_bytes - 4) / 4);
    for (uint32_t i = 0; i < first_batch; i++) {
      *p++ = i + 1 + path_depth * handle_count;
    }

    // Declare length of full buffer to be streamed
    io_container->header->len += handle_count * 4;

    tud_mtp_data_send(io_container);
  } else { // MTP_PHASE_DATA
    const uint32_t data_sent = cb_data->total_xferred_bytes - sizeof(mtp_container_header_t);
    const uint32_t handles_sent = (data_sent - 4) / 4; // how many handles already sent
    const uint32_t handles_remaining = handle_count - handles_sent;
    const uint32_t handles_this_packet = tu_min32(handles_remaining, io_container->payload_bytes / 4);

    uint32_t* p = (uint32_t*) io_container->payload;
    for (uint32_t i = 0; i < handles_this_packet; i++) {
      p[i] = handles_sent + i + 1 + path_depth * handle_count;
    }

    tud_mtp_data_send(io_container);
  }

  return 0;
}

uint16_t babel_file_name[] = {'f','i','l','e',0};
uint16_t babel_dir_name[BABEL_MAX_NAME_LENGTH + 1] = {0};

static int32_t fs_get_object_info(tud_mtp_cb_data_t* cb_data) {
  // `ls -l /<file_in_question>`
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;
  const uint32_t obj_handle = (command->params[0] - 1) % handle_count + 1;
  // struct stat stat_buf;
  // fs_handletable_entry_t *entry = NULL;
  // int retval = fs_stat_handle(&handle_table, obj_handle, &stat_buf, &entry);
  // if (retval != 0) {
  //   ESP_LOGE("MtpImpl", "Failed to stat handle %d: returned %d", obj_handle, retval);
  //   return MTP_RESP_INVALID_OBJECT_HANDLE;
  // }

  // if (obj_handle == handle_count) {
    // snprintf(babel_log + strlen(babel_log), sizeof(babel_log), "get_obj_info\n");
  // }

  uint16_t *entry_name_ptr = babel_file_name;
  bool entry_is_dir = obj_handle != handle_count;

  if (entry_is_dir) {
    uint32_t index = obj_handle - 1;
    for (int i = BABEL_MAX_NAME_LENGTH - 1; i >= 0; i --) {
      babel_dir_name[i] = alphabet[index % BABEL_ALPHABET_LENGTH];
      index /= BABEL_ALPHABET_LENGTH;
    }
    entry_name_ptr = babel_dir_name;
  }

  // uint16_t utf16_filename[MTP_FILENAME_LENGTH];
  // size_t write_count = utf8_to_utf16((uint8_t *)entry_name_ptr, strlen(entry_name_ptr), utf16_filename, MTP_FILENAME_LENGTH);
  // utf16_filename[TU_MIN(write_count, MTP_FILENAME_LENGTH)] = 0;

  mtp_object_info_header_t obj_info_header = {
      .storage_id = SUPPORTED_STORAGE_ID,
      .object_format = entry_is_dir ? MTP_OBJ_FORMAT_ASSOCIATION : MTP_OBJ_FORMAT_TEXT,
      .protection_status = MTP_PROTECTION_STATUS_READ_ONLY,
      .object_compressed_size = -1,
      .thumb_format = MTP_OBJ_FORMAT_UNDEFINED,
      .thumb_compressed_size = 0,
      .thumb_pix_width = 0,
      .thumb_pix_height = 0,
      .image_pix_width = 0,
      .image_pix_height = 0,
      .image_bit_depth = 0,
      .parent_object = 0,
      .association_type = entry_is_dir ? MTP_ASSOCIATION_GENERIC_FOLDER : MTP_ASSOCIATION_UNDEFINED,
      .association_desc = 0,
      .sequence_number = 0};
  mtp_container_add_raw(io_container, &obj_info_header, sizeof(obj_info_header));
  mtp_container_add_string(io_container, entry_name_ptr);
  mtp_container_add_cstring(io_container, FS_FIXED_DATETIME);
  mtp_container_add_cstring(io_container, FS_FIXED_DATETIME);
  mtp_container_add_cstring(io_container, ""); // keywords, not used
  tud_mtp_data_send(io_container);
  // MTP_ESP_LOG("MtpImpl", "Reported %d: %s, size=%d", obj_handle, entry->name, stat_buf.st_size);

  return 0;
}

static int32_t fs_get_object(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;
  const uint32_t obj_handle = command->params[0];

  if (cb_data->phase == MTP_PHASE_COMMAND) {
    // snprintf(babel_log + strlen(babel_log), sizeof(babel_log), "get_obj %lu\n", obj_handle);
    // If file contents is larger than CFG_TUD_MTP_EP_BUFSIZE, data may only partially is added here
    // the rest will be sent in tud_mtp_data_more_cb
    // Rants: TinyUSB MTP design is slightly annoying, it assumed your entire file to transfer is
    // always available in memory space, so he can write their own example code easier. But when you
    // work with reading file live from filesystem, it's awkward in the sense that if your file is
    // longer than the first packet you have to pretend to have added 100KB into the buffer (which
    // it can't hold, but the whole thing will pretend to have done that, records how many bytes
    // are actually transmitted, and next time in data phase you calculate the length of the rest)
    // We use a buffer that's large enough: our MTP packet is 512B, and the first time we transmit
    // something back, we need to send metadata back, so if we read a whole packet it's definitely
    // not gonna fit in the MTP packet's remaining space if our file is larger than the free space,
    // and when the file's smaller than that we're totally fine then.
    // fs_append_output(obj_handle - 1);
    size_t file_size = output_file_len;
    // Pass file_size (not first_chunk) so add_raw records the full transfer length in header->len,
    // even though it only copies min(file_size, remaining_buf) bytes into the packet buffer.
    uint32_t bytes_queued = mtp_container_add_raw(io_container, output_file, file_size);
    // if (bytes_queued >= output_file_len) {
    //   fs_reverse_output(obj_handle - 1);
    // }
    tud_mtp_data_send(io_container);
  } else if (cb_data->phase == MTP_PHASE_DATA) {
    // continue sending remaining data: file contents offset is xferred byte minus header size
    const uint32_t offset = cb_data->total_xferred_bytes - sizeof(mtp_container_header_t);
    const uint32_t xact_len = tu_min32(output_file_len - offset, io_container->payload_bytes);
    uint8_t *write_ptr = io_container->payload;
    memcpy(write_ptr, output_file + offset, xact_len);
    tud_mtp_data_send(io_container);
    // if (offset + xact_len >= output_file_len) {
    //   fs_reverse_output(obj_handle - 1);
    // }
  }

  return 0;

}

static int32_t fs_send_object_info(tud_mtp_cb_data_t* cb_data) {
  return MTP_RESP_OPERATION_NOT_SUPPORTED;

  // const mtp_container_command_t* command = cb_data->command_container;
  // mtp_container_info_t* io_container = &cb_data->io_container;
  // const uint32_t storage_id = command->params[0];
  // const uint32_t parent_handle = command->params[1]; // folder handle, 0xFFFFFFFF is root
  // (void) parent_handle;

  // MTP_ESP_LOG("MtpImpl", "fs_send_object_info");

  // if (!is_session_opened) {
  //   MTP_ESP_LOG("MtpImpl", "%s: session not open", __func__);
  //   return MTP_RESP_SESSION_NOT_OPEN;
  // }
  // if (storage_id != 0xFFFFFFFF && storage_id != SUPPORTED_STORAGE_ID) {
  //   MTP_ESP_LOG("MtpImpl", "%s: invalid storage ID %08X", __func__, storage_id);
  //   return MTP_RESP_INVALID_STORAGE_ID;
  // }

  // if (cb_data->phase == MTP_PHASE_COMMAND) {
  //   MTP_ESP_LOG("MtpImpl", "%s: command phase, receive first", __func__);
  //   tud_mtp_data_receive(io_container);
  // } else if (cb_data->phase == MTP_PHASE_DATA) {
  //   mtp_object_info_header_t* obj_info = (mtp_object_info_header_t*) io_container->payload;
  //   if (obj_info->storage_id != 0 && obj_info->storage_id != SUPPORTED_STORAGE_ID) {
  //     MTP_ESP_LOG("MtpImpl", "%s: data phase: invalid storage ID %08X", __func__, obj_info->storage_id);
  //     return MTP_RESP_INVALID_STORAGE_ID;
  //   }

  //   // 0xFFFFFFFF is root
  //   fs_handle_t parent_handle = (obj_info->parent_object == 0xFFFFFFFF ? 0 : obj_info->parent_object);

  //   if (parent_handle != 0) {
  //     fs_handletable_entry_t *entry;
  //     struct stat stat;
  //     int retval = fs_stat_handle(&handle_table, parent_handle, &stat, &entry);
  //     if (retval != 0 || (stat.st_mode & S_IFDIR) == 0) {
  //       ESP_LOGE("MtpImpl", "Invalid parent %X: stat retval %d, st_mode %X", parent_handle, retval, stat.st_mode);
  //       return MTP_RESP_INVALID_PARENT_OBJECT;
  //     }
  //   }

  //   if (obj_info->association_type == MTP_ASSOCIATION_UNDEFINED) {
  //     // Regular file
  //     if (!fs_can_create_file(&handle_table, obj_info->object_compressed_size)) {
  //       return MTP_RESP_STORE_FULL;
  //     }
  //     char filename[MTP_FILENAME_LENGTH];
  //     uint8_t* filename_buf = io_container->payload + sizeof(mtp_object_info_header_t);
  //     uint8_t filename_len = *filename_buf;
  //     size_t u8_filename_len = utf16_to_utf8((const utf16_t *)(filename_buf + 1),
  //                                          filename_len,
  //                                          (utf8_t *)filename,
  //                                          MTP_FILENAME_LENGTH);
  //     MTP_ESP_LOG("MtpImpl", "UTF16->UTF8 Conversion: expected UTF16 length: %d, output %d bytes, string:[%s]",
  //              filename_len, u8_filename_len, filename);
  //     fs_create_file(&handle_table, parent_handle, filename);
  //     // Here the current_file_size is used to hold the length-to-receive value till the send_object phase
  //     current_file_size = obj_info->object_compressed_size;
  //   } else if (obj_info->association_type == MTP_ASSOCIATION_GENERIC_FOLDER) {
  //     // Folder
  //     if (parent_handle != 0) {
  //       ESP_LOGE("MtpImpl", "Attempting to create folder in folder %d", parent_handle);
  //       return MTP_RESP_INVALID_PARENT_OBJECT;
  //     }
  //     char dir_path[100] = "/littlefs/";
  //     char *dir_name = dir_path + strlen(dir_path);
  //     uint8_t* filename_buf = io_container->payload + sizeof(mtp_object_info_header_t);
  //     uint8_t filename_len = *filename_buf;
  //     utf16_to_utf8((const utf16_t *)(filename_buf + 1),
  //                   filename_len,
  //                   (utf8_t *)dir_name,
  //                   sizeof(dir_path) - (dir_name - dir_path) - 1);
  //     mkdir(dir_path, 0777);
  //   } else {
  //     ESP_LOGE("MtpImpl", "Attempting to create unsupported association: %d", obj_info->association_type);
  //     return MTP_RESP_INVALID_PARAMETER;
  //   }
  //   // ignore date created/modified/keywords
  // }

  // return 0;
}

static int32_t fs_send_object(tud_mtp_cb_data_t* cb_data) {
  return MTP_RESP_OPERATION_NOT_SUPPORTED;
  // mtp_container_info_t* io_container = &cb_data->io_container;
  // if (current_file == NULL || current_handle == FS_INVALID_HANDLE) {
  //   return MTP_RESP_INVALID_OBJECT_HANDLE;
  // }

  // if (cb_data->phase == MTP_PHASE_COMMAND) {
  //   io_container->header->len += current_file_size;
  //   tud_mtp_data_receive(io_container);
  // } else if (cb_data->phase == MTP_PHASE_DATA) {
  //   // file contents offset is total xferred minus header size minus last received chunk
  //   // const uint32_t offset = cb_data->total_xferred_bytes - sizeof(mtp_container_header_t) - io_container->payload_bytes;
  //   // memcpy(f->data + offset, io_container->payload, io_container->payload_bytes);
  //   fwrite(io_container->payload, 1, io_container->payload_bytes, current_file);
  //   MTP_ESP_LOG("MtpImpl", "%s: data phase, written %d bytes to file", __func__, io_container->payload_bytes);
  //   if (cb_data->total_xferred_bytes - sizeof(mtp_container_header_t) < current_file_size) {
  //     MTP_ESP_LOG("MtpImpl", "%s: Starting new reception, %d bytes to go",
  //              __func__,
  //              current_file_size - cb_data->total_xferred_bytes + sizeof(mtp_container_header_t));
  //     MTP_ESP_LOG("MtpImpl", "%s: pcontainer->header->len = %d", __func__, io_container->header->len);
  //     tud_mtp_data_receive(io_container);
  //   } else {
  //     MTP_ESP_LOG("MtpImpl", "%s: File write completed, closing", __func__);
  //     fs_close_handle(current_handle, current_file);
  //   }
  // } else {
  //   ESP_LOGE("MtpImpl", "%s: Unknown phase %d", __func__, cb_data->phase);
  // }
  // MTP_ESP_LOG("MtpImpl", "%s: leaving", __func__);

  // return 0;
}

static int32_t fs_delete_object(tud_mtp_cb_data_t* cb_data) {
  return MTP_RESP_OPERATION_NOT_SUPPORTED;
  // const mtp_container_command_t* command = cb_data->command_container;
  // const uint32_t obj_handle = command->params[0];
  // const uint32_t obj_format = command->params[1]; // optional
  // (void) obj_format;

  // if (!is_session_opened) {
  //   return MTP_RESP_SESSION_NOT_OPEN;
  // }

  // char pathbuf[200];
  // if (!fs_path_from_handle(&handle_table, obj_handle, pathbuf, sizeof(pathbuf))) {
  //   return MTP_RESP_INVALID_OBJECT_HANDLE;
  // }

  // struct stat stat;
  // fs_handletable_entry_t *entry;
  // int retval = fs_stat_handle(&handle_table, obj_handle, &stat, &entry);
  // if (retval != 0) {
  //   ESP_LOGE("MtpImpl", "fs_delete_object failed to stat %s: %d", pathbuf, retval);
  //   return MTP_RESP_GENERAL_ERROR;
  // }

  // if (stat.st_mode & S_IFDIR) {
  //   // TODO: Is a directory, delete descendants
  //   return MTP_RESP_OPERATION_NOT_SUPPORTED;
  // } else {
  //   unlink(pathbuf);
  //   fs_delete_handle(&handle_table, obj_handle);
  // }

  // return MTP_RESP_OK;
}

int32_t fs_format_store(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  const uint32_t storage_id = command->params[0];

  if (!is_session_opened) {
    MTP_ESP_LOG("MtpImpl", "%s: session not open", __func__);
    return MTP_RESP_SESSION_NOT_OPEN;
  }
  if (storage_id != 0xFFFFFFFF && storage_id != SUPPORTED_STORAGE_ID) {
    MTP_ESP_LOG("MtpImpl", "%s: invalid storage ID %08X", __func__, storage_id);
    return MTP_RESP_INVALID_STORAGE_ID;
  }
  if (current_handle != FS_INVALID_HANDLE || current_file != NULL) {
    MTP_ESP_LOG("MtpImpl", "%s: storage %08X busy", __func__, storage_id);
    return MTP_RESP_DEVICE_BUSY;
  }
  // esp_err_t rc = esp_littlefs_format("littlefs");
  // if (rc == ESP_OK) {
  //   MTP_ESP_LOG("MtpImpl", "%s: format succeeded", __func__);

  //   handle_self_inc = 0;
  //   fs_handletable_regenerate(&handle_table);

    return MTP_RESP_OK;
  // } else {
  //   MTP_ESP_LOG("MtpImpl", "%s: format failed (rc = %d)", __func__, rc);

  //   return MTP_RESP_OPERATION_NOT_SUPPORTED;
  // }
}
