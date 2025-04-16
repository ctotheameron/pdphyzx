#include <glob.h>
#include <libgen.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NOB_IMPLEMENTATION
#define NOB_STRIP_PREFIX
#define NOB_EXPERIMENTAL_DELETE_OLD
#include "nob.h"

#define H_GLOB "src/includes/*.h"
#define C_GLOB "src/*.c"
#define SRC_GLOB "{src/**/*.c,src/**/*.h,examples/**/*.c, examples/**/*.h}"

#define SYSCALLS_STUB_FILE "syscalls_stub.c"

// - Compiler flags ------------------------------------------------------------
#define COMMON_CFLAGS                                                          \
  "-Wall", "-Wextra", "-pedantic", "-std=gnu11", "-Wstrict-prototypes",        \
      "-Wno-unknown-pragmas", "-Wdouble-promotion", "-DTARGET_EXTENSION=1"

//- Debug/Release --------------------------------------------------------------
#define DEV_FLAGS "-g", "-O0", "-DDEBUG"
#define RELEASE_FLAGS "-O2", "-DNDEBUG"

//- Target-specific ------------------------------------------------------------
#define SIMULATOR_FLAGS_COMMON                                                 \
  "-shared", "-fPIC", "-cl-single-precision-constant", "-DTARGET_SIMULATOR=1"

#if defined(__linux__)
#define SIMULATOR_FLAGS SIMULATOR_FLAGS_COMMON,
#else // Assume macOS otherwise
#define SIMULATOR_FLAGS SIMULATOR_FLAGS_COMMON, "-arch", "arm64",
#endif

#define DEVICE_FLAGS                                                           \
  "-DTARGET_PLAYDATE=1", "-mthumb", "-mcpu=cortex-m7", "-mfloat-abi=hard",     \
      "-mfpu=fpv5-sp-d16", "-D__FPU_USED=1", "-falign-functions=16",           \
      "-fomit-frame-pointer", "-gdwarf-2", "-ffunction-sections",              \
      "--single-precision-constant", "-fdata-sections", "-mword-relocations",  \
      "-fno-common", "-nostartfiles", "-fverbose-asm",                         \
      "-D__HEAP_SIZE=8388208", "-D__STACK_SIZE=61800",                         \
      "-Wl,--entry=eventHandlerShim"
//------------------------------------------------------------------------------

#define BUILD_DIR "build"
#define DIST_DIR "dist"

const char *SDK;
const char *SDK_C;

bool setPlaydateSDK(void) {
  const char *envSdkPath = getenv("PLAYDATE_SDK_PATH");

  if (!envSdkPath || strlen(envSdkPath) <= 0) {
    nob_log(ERROR, "PLAYDATE_SDK_PATH is not set");
    return false;
  }

  nob_log(INFO, "PLAYDATE_SDK_PATH: %s", envSdkPath);
  SDK = envSdkPath;
  SDK_C = temp_sprintf("%s/C_API", SDK);
  return true;
}

//==============================================================================
// CLI
//==============================================================================

typedef enum {
  ALL,
  BUILD,
  BUNDLE,
  TEST,
  RUN,
  HELP,
  UNKNOWN,
} Command;

typedef enum { DEV, RELEASE } Mode;
typedef enum { SIMULATOR, DEVICE } Target;

typedef struct {
  Command command;
  Mode mode;
  Target target;
  bool skipBundle;
  const char *example;
} Options;

const char *modeName(Mode mode) {
  switch (mode) {
  case DEV:
    return "dev";
  case RELEASE:
    return "release";
  default:
    return "unknown";
  }
}

const char *targetName(Target target) {
  switch (target) {
  case SIMULATOR:
    return "simulator";
  case DEVICE:
    return "device";
  default:
    assert(0 && "Unknown target");
  }
}

const char *targetExt(Target target) {
  switch (target) {
  case SIMULATOR:
    return "dylib";
  case DEVICE:
    return "elf";
  default:
    assert(0 && "Unknown target");
  }
}

Command parseCommand(const char *commandStr) {
  if (strlen(commandStr) == 0 || strcmp(commandStr, "all") == 0) {
    return ALL;
  }

  if (strcmp(commandStr, "build") == 0) {
    return BUILD;
  }

  if (strcmp(commandStr, "bundle") == 0) {
    return BUNDLE;
  }

  if (strcmp(commandStr, "run") == 0) {
    return RUN;
  }

  if (strcmp(commandStr, "test") == 0) {
    return TEST;
  }

  if (strcmp(commandStr, "help") == 0) {
    return HELP;
  }

  nob_log(ERROR, "Unknown command: %s", commandStr);
  return UNKNOWN;
}

Options parseBuildOptions(int argc, char **argv) {
  Options options = {
      .command = ALL, .mode = DEV, .target = SIMULATOR, .skipBundle = false};

  if (argc > 0) {
    const char *firstArg = argv[0];

    // Only treat as command if it doesn't start with '-'
    if (firstArg[0] != '-') {
      const char *commandStr = shift(argv, argc);
      options.command = parseCommand(commandStr);

      // Check for an example name as the next argument (if not starting with -)
      if (argc > 0 && argv[0][0] != '-') {
        options.example = shift(argv, argc);
      }
    }
  }

  while (argc > 0) {
    const char *arg = shift(argv, argc);

    // Mode flags
    if (strcmp(arg, "--dev") == 0 || strcmp(arg, "-d") == 0) {
      options.mode = DEV;
      continue;
    }

    if (strcmp(arg, "--release") == 0 || strcmp(arg, "-r") == 0) {
      options.mode = RELEASE;
      continue;
    }

    // Target flags
    if (strcmp(arg, "--target=simulator") == 0 ||
        strcmp(arg, "-t=simulator") == 0 || strcmp(arg, "-t=sim") == 0) {
      options.target = SIMULATOR;
      continue;
    }

    if (strcmp(arg, "--target=device") == 0 || strcmp(arg, "-t=device") == 0 ||
        strcmp(arg, "-t=dev") == 0) {
      options.target = DEVICE;
      continue;
    }

    if (strcmp(arg, "--no-bundle") == 0) {
      options.skipBundle = true;
      continue;
    }

    nob_log(WARNING, "Ignoring unknown option: %s", arg);
  }

  if (options.command == RUN && !options.example) {
    nob_log(ERROR, "No example specified for run command");
    options.command = UNKNOWN;
  }

  return options;
}

//==============================================================================
// Build
//==============================================================================

bool createGitignore(const char *dir) {
  const char *gitignorePath = temp_sprintf("%s/.gitignore", dir);
  const char *content = "*\n";
  return write_entire_file(gitignorePath, content, strlen(content));
}

bool ensureBuildDir(Target target, const char *example) {
  // Create necessary directories
  if (!mkdir_if_not_exists(BUILD_DIR) ||
      !mkdir_if_not_exists(BUILD_DIR "/tmp") || !createGitignore(BUILD_DIR)) {
    return false;
  }

  // Create target directory
  const char *targetDir = temp_sprintf("%s/%s", BUILD_DIR, targetName(target));

  if (!mkdir_if_not_exists(targetDir)) {
    return false;
  }

  // Create example-specific directories if an example is specified
  if (example) {
    if (!mkdir_if_not_exists(temp_sprintf(BUILD_DIR "/tmp/%s", example)) ||
        !mkdir_if_not_exists(temp_sprintf("%s/%s", targetDir, example))) {
      return false;
    }
  }

  return true;
}

void configureSimulatorBuild(Cmd *cmd) {
  cmd_append(cmd, "clang");
  cmd_append(cmd, SIMULATOR_FLAGS);
}

void configureDeviceBuild(Cmd *cmd) {
  cmd_append(cmd, "arm-none-eabi-gcc");
  cmd_append(cmd, DEVICE_FLAGS);

  // Add link map for device builds
  char *linkMap = temp_sprintf("%s/buildsupport/link_map.ld", SDK_C);

  cmd_append(cmd, "-T", linkMap);
  cmd_append(cmd,
             "-Wl,-Map=build/"
             "game.map,--cref,--gc-sections,--no-warn-mismatch,--emit-relocs");

  // Add setup.c from SDK
  char *setupC = temp_sprintf("%s/buildsupport/setup.c", SDK_C);
  cmd_append(cmd, setupC);
}

// Write syscall stubs to file
bool ensureSyscallStubsFile(void) {
  const char *stubsPath = temp_sprintf("%s/" SYSCALLS_STUB_FILE, BUILD_DIR);

  if (file_exists(stubsPath)) {
    return true;
  }

  nob_log(INFO, "Creating " SYSCALLS_STUB_FILE " for device debug builds");

  char *stubs = "#include <sys/stat.h>\n"
                "#include <errno.h>\n"
                "#include <stdint.h>\n\n"
                "// System call stubs for embedded environment\n\n"
                "int _close(int file) {\n"
                "    (void)file;\n"
                "    return -1;\n"
                "}\n\n"
                "int _fstat(int file, struct stat *st) {\n"
                "    (void)file;\n"
                "    st->st_mode = S_IFCHR;\n"
                "    return 0;\n"
                "}\n\n"
                "int _isatty(int file) {\n"
                "    (void)file;\n"
                "    return 1;\n"
                "}\n\n"
                "int _lseek(int file, int ptr, int dir) {\n"
                "    (void)file;\n"
                "    (void)ptr;\n"
                "    (void)dir;\n"
                "    return 0;\n"
                "}\n\n"
                "int _read(int file, char *ptr, int len) {\n"
                "    (void)file;\n"
                "    (void)ptr;\n"
                "    (void)len;\n"
                "    return 0;\n"
                "}\n\n"
                "int _write(int file, char *ptr, int len) {\n"
                "    (void)file;\n"
                "    (void)ptr;\n"
                "    (void)len;\n"
                "    return 0;\n"
                "}\n\n"
                "void _exit(int status) {\n"
                "    (void)status;\n"
                "    while (1);\n"
                "}\n\n"
                "int _getpid(void) {\n"
                "    return 1;\n"
                "}\n\n"
                "int _kill(int pid, int sig) {\n"
                "    (void)pid;\n"
                "    (void)sig;\n"
                "    errno = EINVAL;\n"
                "    return -1;\n"
                "}\n";

  return write_entire_file(temp_sprintf("%s/" SYSCALLS_STUB_FILE, BUILD_DIR),
                           stubs, strlen(stubs));
}

bool configureBuild(Cmd *cmd, Mode mode, Target target) {
  switch (target) {
  case SIMULATOR:
    configureSimulatorBuild(cmd);
    break;
  case DEVICE:
    configureDeviceBuild(cmd);
  }

  cmd_append(cmd, COMMON_CFLAGS, temp_sprintf("-I%s", SDK_C));

  switch (mode) {
  case DEV:
    cmd_append(cmd, DEV_FLAGS);
    break;
  case RELEASE:
    cmd_append(cmd, RELEASE_FLAGS);
    break;
  }

  return true;
}

bool genLspInfo(void) {
  nob_log(INFO, "Generating compile_commands.json for LSP...");

  // Always use simulator and debug mode for LSP support
  Cmd lspCmd = {0};
  if (!configureBuild(&lspCmd, DEV, SIMULATOR)) {
    return false;
  }

  // Add source includes
  cmd_append(&lspCmd, "-Isrc/includes");

  // Get source files
  glob_t srcGlob;
  if (glob(SRC_GLOB, GLOB_BRACE | GLOB_NOSORT, NULL, &srcGlob) != 0) {
    nob_log(ERROR, "No source files found");
    return false;
  }

  char cwd[1024];
  if (getcwd(cwd, sizeof(cwd)) == NULL) {
    nob_log(ERROR, "Failed to get current working directory");
    return false;
  }

  if (!mkdir_if_not_exists(BUILD_DIR)) {
    return false;
  }

  String_Builder lspJson = {0};

  // Start JSON array
  sb_append_cstr(&lspJson, "[\n");

  bool first = true;

  for (size_t i = 0; i < srcGlob.gl_pathc; ++i) {
    char *src = srcGlob.gl_pathv[i];
    char *obj = temp_sprintf("%s/%s.o", BUILD_DIR, src);

    // Clone the build command for this file
    Cmd fileCmd = {0};
    for (size_t j = 0; j < lspCmd.count; j++) {
      cmd_append(&fileCmd, lspCmd.items[j]);
    }

    // Add file-specific output and source
    cmd_append(&fileCmd, "-o", obj, src);

    // Convert command to string for JSON
    String_Builder cmdStr = {0};
    for (size_t j = 0; j < fileCmd.count; ++j) {
      if (strchr(fileCmd.items[j], ' ')) {
        sb_append_cstr(&cmdStr, "\"");
        sb_append_cstr(&cmdStr, fileCmd.items[j]);
        sb_append_cstr(&cmdStr, "\"");
      } else {
        sb_append_cstr(&cmdStr, fileCmd.items[j]);
      }

      if (j < fileCmd.count - 1) {
        sb_append_cstr(&cmdStr, " ");
      }
    }

    // Write JSON entry
    if (!first) {
      sb_append_cstr(&lspJson, ",\n");
    }

    first = false;

    sb_append_cstr(&lspJson, "  {\n");
    sb_appendf(&lspJson, "    \"directory\": \"%s\",\n", cwd);
    sb_appendf(&lspJson, "    \"command\": \"%s\",\n", cmdStr.items);
    sb_appendf(&lspJson, "    \"file\": \"%s/%s\",\n", cwd, src);
    sb_appendf(&lspJson, "    \"output\": \"%s\"\n", obj);
    sb_append_cstr(&lspJson, "  }");
  }

  // End JSON array
  sb_append_cstr(&lspJson, "\n]\n");

  // Write
  char *json_path = temp_sprintf("%s/compile_commands.json", BUILD_DIR);

  if (!write_entire_file(json_path, lspJson.items, lspJson.count)) {
    nob_log(ERROR, "Failed to write %s", json_path);
    return false;
  }

  nob_log(INFO, "Generated %s", json_path);
  return true;
}

bool build(Options options) {
  Mode mode = options.mode;
  Target target = options.target;
  const char *example = options.example;

  // If no example specified, build all examples in the directory
  if (!example) {
    nob_log(INFO, "Building all examples...");

    // Get all directories in examples/
    glob_t exampleGlob;
    if (glob("examples/*", 0, NULL, &exampleGlob) != 0) {
      nob_log(ERROR, "No example directories found");
      return false;
    }

    bool result = true;
    for (size_t i = 0; i < exampleGlob.gl_pathc; ++i) {
      char *exampleDir = exampleGlob.gl_pathv[i];

      // Skip if not a directory
      if (get_file_type(exampleDir) != FILE_DIRECTORY) {
        continue;
      }

      char *exampleName = basename(exampleDir);

      nob_log(INFO, "Building example: %s", exampleName);

      // Create temporary options with this example
      Options exampleOpts = options;
      exampleOpts.example = exampleName;

      if (!build(exampleOpts)) {
        nob_log(ERROR, "Failed to build example: %s", exampleName);
        result = false;
      }
    }

    return result;
  }

  nob_log(INFO, "Running %s build for %s (example: %s)...", modeName(mode),
          targetName(target), example);

  if (!ensureBuildDir(target, example)) {
    return false;
  }

  // Create syscall stubs file for device debug builds if needed
  if ((target == DEVICE && mode == DEV) && !ensureSyscallStubsFile()) {
    return false;
  }

  // Determine output file path for this example
  const char *outDir =
      temp_sprintf(BUILD_DIR "/%s/%s", targetName(target), example);

  const char *out = temp_sprintf("%s/pdex.%s", outDir, targetExt(target));

  // Get source files - check in examples directory first
  char *exampleSrcGlob = temp_sprintf("examples/%s/*.c", example);
  glob_t srcGlob;
  if (glob(exampleSrcGlob, GLOB_BRACE | GLOB_NOSORT, NULL, &srcGlob) != 0) {
    nob_log(ERROR, "No source files found for example: %s", example);
    return false;
  }

  // Configure the build
  Cmd cmd = {0};
  if (!configureBuild(&cmd, mode, target)) {
    return false;
  }

  // Add dist
  cmd_append(&cmd, "-I", DIST_DIR);

  // Add syscalls stub file for device debug builds
  if (target == DEVICE && mode == DEV) {
    cmd_append(&cmd, temp_sprintf("%s/" SYSCALLS_STUB_FILE, BUILD_DIR));
  }

  // Add all source files
  for (size_t i = 0; i < srcGlob.gl_pathc; ++i) {
    cmd_append(&cmd, srcGlob.gl_pathv[i]);
  }

  // Specify output file
  cmd_append(&cmd, "-o", out);

  // Run compilation
  if (!cmd_run_sync_and_reset(&cmd)) {
    return false;
  }

  // Copy assets directory to the tmp directory (if they exist)
  char *assetsDir = temp_sprintf("examples/%s/assets", example);
  char *tmpDir = temp_sprintf(BUILD_DIR "/tmp/%s", example);

  bool should_copy =
      nob_file_exists(assetsDir) && get_file_type(assetsDir) == FILE_DIRECTORY;

  if (should_copy && !copy_directory_recursively(assetsDir, tmpDir)) {
    nob_log(WARNING, "Failed to copy assets from %s", assetsDir);
    return false;
  }

  // Post-processing
  switch (target) {
  case SIMULATOR:
    // Copy simulator dylib to example's tmp folder
    copy_file(out, temp_sprintf("%s/pdex.dylib", tmpDir));
    break;
  case DEVICE:
    // Process device ELF: strip unneeded symbols
    cmd_append(&cmd, "arm-none-eabi-strip", "--strip-unneeded", "-R",
               ".comment", "-g", out, "-o",
               temp_sprintf("%s/pdex.elf", tmpDir));

    if (!cmd_run_sync_and_reset(&cmd)) {
      return false;
    }
  }

  // Copy example's pdxinfo file
  copy_file(temp_sprintf("examples/%s/pdxinfo", example),
            temp_sprintf("%s/pdxinfo", tmpDir));

  // Package with pdc
  cmd_append(&cmd, temp_sprintf("%s/bin/pdc", SDK), "-sdkpath", SDK, tmpDir,
             temp_sprintf("%s/pdphyzx_%s.pdx", BUILD_DIR, example));

  return cmd_run_sync_and_reset(&cmd);
}

typedef struct {
  const char *path;
  char **dependencies;
  size_t depCount;
  bool visited;
  bool inCurrentPath;
} HeaderFile;

static void visitDep(size_t idx, HeaderFile *headers, size_t headerCount,
                     const char **sortedFiles, size_t *sortedCount,
                     bool *hasCycle) {

  if (headers[idx].inCurrentPath) {
    nob_log(ERROR, "Circular dependency detected in %s", headers[idx].path);
    *hasCycle = true;
    return;
  }

  if (headers[idx].visited) {
    return;
  }

  headers[idx].inCurrentPath = true;

  // Visit dependencies
  for (size_t i = 0; i < headers[idx].depCount; i++) {
    char *dep = headers[idx].dependencies[i];

    // Skip standard library includes
    if (dep[0] == '<') {
      continue;
    }

    // Find dependency in our list
    for (size_t j = 0; j < headerCount; j++) {
      char *pathCpy = strdup(headers[j].path);
      char *filename = basename(pathCpy);

      if (strcmp(filename, dep) == 0) {
        visitDep(j, headers, headerCount, sortedFiles, sortedCount, hasCycle);
        break;
      }
    }
  }

  headers[idx].visited = true;
  headers[idx].inCurrentPath = false;
  sortedFiles[(*sortedCount)++] = headers[idx].path;
}

bool findAndParseHeaders(HeaderFile **headers, size_t *headerCount) {
  glob_t headerGlob;
  if (glob(H_GLOB, GLOB_BRACE | GLOB_NOSORT, NULL, &headerGlob) != 0) {
    nob_log(ERROR, "No header files found");
    return false;
  }

  *headerCount = headerGlob.gl_pathc;
  *headers = calloc(headerGlob.gl_pathc, sizeof(HeaderFile));
  if (!*headers) {
    nob_log(ERROR, "Memory allocation failed");
    return false;
  }

  // Parse dependencies
  for (size_t i = 0; i < headerGlob.gl_pathc; ++i) {
    (*headers)[i].path = headerGlob.gl_pathv[i];
    (*headers)[i].dependencies = NULL;
    (*headers)[i].depCount = 0;

    // Read entire file content
    String_Builder fileContent = {0};
    if (!read_entire_file((*headers)[i].path, &fileContent)) {
      continue;
    }

    // Count dependencies first
    String_View content = sb_to_sv(fileContent);
    String_View line;
    size_t count = 0;

    String_View remaining = content;
    while (remaining.count > 0) {
      line = sv_chop_by_delim(&remaining, '\n');
      if (strstr(temp_sv_to_cstr(line), "#include \"")) {
        count++;
      }
    }

    (*headers)[i].depCount = count;

    if ((*headers)[i].depCount > 0) {
      (*headers)[i].dependencies =
          calloc((*headers)[i].depCount, sizeof(char *));

      size_t dep_idx = 0;
      remaining = content;

      while (remaining.count > 0 && dep_idx < (*headers)[i].depCount) {
        line = sv_chop_by_delim(&remaining, '\n');
        const char *line_str = temp_sv_to_cstr(line);

        if (!strstr(line_str, "#include \"")) {
          continue;
        }

        const char *start = strchr(line_str, '"');
        if (!start) {
          continue;
        }

        start += 1;

        const char *end = strrchr(line_str, '"');
        if (!end || start >= end) {
          continue;
        }

        String_View dep = sv_from_parts(start, end - start);
        char *depName = calloc(dep.count + 1, sizeof(char));
        memcpy(depName, dep.data, dep.count);
        (*headers)[i].dependencies[dep_idx++] = depName;
      }
    }
  }

  return true;
}

bool topologicallySortHeaders(HeaderFile *headers, size_t headerCount,
                              const char ***sortedFiles, size_t *sortedCount) {
  *sortedFiles = calloc(headerCount, sizeof(char *));
  *sortedCount = 0;

  bool hasCycle = false;

  for (size_t i = 0; i < headerCount; i++) {
    if (headers[i].visited) {
      continue;
    }

    visitDep(i, headers, headerCount, *sortedFiles, sortedCount, &hasCycle);
  }

  if (hasCycle) {
    nob_log(ERROR, "Circular dependencies detected in header files");
    return false;
  }

  return true;
}

bool collectStandardIncludes(char **stdIncludes, size_t *includeCount) {
  nob_log(INFO, "Collecting standard library includes...");

  *includeCount = 0;

  // Process all source files to collect standard includes
  glob_t allGlob;
  if (glob("{" C_GLOB "," H_GLOB "}", GLOB_BRACE | GLOB_NOSORT, NULL,
           &allGlob) != 0) {
    nob_log(ERROR, "Failed to find source files for include collection");
    return false;
  }

  // First pass: collect all unique standard library includes from all files
  for (size_t i = 0; i < allGlob.gl_pathc; ++i) {
    String_Builder fileContent = {0};
    if (!read_entire_file(allGlob.gl_pathv[i], &fileContent)) {
      continue;
    }

    String_View content = sb_to_sv(fileContent);
    String_View line;
    String_View remaining = content;

    while (remaining.count > 0) {
      line = sv_chop_by_delim(&remaining, '\n');
      line = sv_trim_right(line); // Remove trailing whitespace

      // Look for standard includes with the pattern: #include <header.h>
      if (sv_starts_with(line, sv_from_cstr("#include <")) &&
          sv_end_with(line, ">")) {

        // Check if this include is already in our list
        bool duplicate = false;
        for (size_t j = 0; j < *includeCount; j++) {
          if (sv_eq(sv_from_cstr(stdIncludes[j]), line)) {
            duplicate = true;
            break;
          }
        }

        if (!duplicate && *includeCount < 100) {
          stdIncludes[*includeCount] = strdup(temp_sv_to_cstr(line));
          (*includeCount)++;
        }
      }
    }
  }

  return true;
}

static bool appendImplementationCode(String_Builder *output) {
  sb_append_cstr(output, "\n#ifdef PDPHYZX_IMPLEMENTATION\n\n");

  // Implementation files don't need topological sorting as they depend on
  // headers
  glob_t implGlob;
  if (glob(C_GLOB, GLOB_BRACE | GLOB_NOSORT, NULL, &implGlob) != 0) {
    nob_log(ERROR, "Failed to find implementation files");
    return false;
  }

  nob_log(INFO, "Adding implementation code...");

  for (size_t i = 0; i < implGlob.gl_pathc; ++i) {
    String_Builder impl = {0};
    if (!read_entire_file(implGlob.gl_pathv[i], &impl)) {
      continue;
    }

    // Add file comment
    sb_append_cstr(output, "// Implementation from ");
    sb_append_cstr(output, implGlob.gl_pathv[i]);
    sb_append_cstr(output, "\n");

    String_View content = sb_to_sv(impl);
    String_View line;
    String_View remaining = content;

    bool inIncludes = false;

    while (remaining.count > 0) {
      line = sv_chop_by_delim(&remaining, '\n');
      const char *lineStr = temp_sv_to_cstr(line);

      // Skip includes
      if (strstr(lineStr, "#include")) {
        inIncludes = true;
        continue;
      }

      if (inIncludes) {
        if (line.count > 0 && line.data[0] != '\n') {
          continue;
        }
        inIncludes = false;
      }

      sb_append_buf(output, line.data, line.count);
      sb_append_cstr(output, "\n");
    }

    sb_append_cstr(output, "\n");
  }

  return true;
}

bool buildOutputContent(String_Builder *o, const char **files, size_t count,
                        char **stdIncludes, size_t includeCount) {
  // Write header
  sb_append_cstr(o, "/**\n");
  sb_append_cstr(o, " * pdphyzx.h - Playdate Physics Engine\n");
  sb_append_cstr(o, " * Single-header version generated by build script\n");
  sb_append_cstr(o, " */\n\n");
  sb_append_cstr(o, "#ifndef PDPHYZX_H\n");
  sb_append_cstr(o, "#define PDPHYZX_H\n\n");

  // Write all standard library includes at the beginning
  for (size_t i = 0; i < includeCount; i++) {
    sb_append_cstr(o, stdIncludes[i]);
    sb_append_cstr(o, "\n");
  }

  sb_append_cstr(o, "\n");
  // Add forward declaration for PlaydateAPI for LSP support
  sb_append_cstr(o, "#ifndef PlaydateAPI\n");
  sb_append_cstr(o, "typedef struct PlaydateAPI PlaydateAPI;\n");
  sb_append_cstr(o, "#endif\n\n");

  // Write forward declarations and API in topologically sorted order
  nob_log(INFO, "Adding API declarations in dependency order...");
  for (size_t i = 0; i < count; ++i) {
    String_Builder header = {0};
    if (!read_entire_file(files[i], &header)) {
      continue;
    }

    String_View content_view = sb_to_sv(header);
    String_View line;
    String_View remaining = content_view;

    bool inIncludes = false;

    while (remaining.count > 0) {
      line = sv_chop_by_delim(&remaining, '\n');

      // Skip include guards
      if (sv_starts_with(line, sv_from_cstr("#ifndef PDPHYZX_")) ||
          sv_starts_with(line, sv_from_cstr("#define PDPHYZX_")) ||
          sv_starts_with(line, sv_from_cstr("#endif // PDPHYZX_")) ||
          sv_starts_with(line, sv_from_cstr("#pragma once"))) {
        continue;
      }

      if (sv_starts_with(line, sv_from_cstr("#include"))) {
        inIncludes = true;
        continue;
      }

      if (inIncludes) {
        if (line.count > 0 && line.data[0] != '\n') {
          continue;
        }

        inIncludes = false;
      }

      sb_append_buf(o, line.data, line.count);
      sb_append_cstr(o, "\n");
    }

    sb_append_cstr(o, "\n");
  }

  // Now add implementation section
  if (!appendImplementationCode(o)) {
    return false;
  }

  // End implementation and header
  sb_append_cstr(o, "\n#endif // PDPHYZX_IMPLEMENTATION\n");
  sb_append_cstr(o, "\n#endif // PDPHYZX_H\n");

  return true;
}

bool bundleHeader(void) {
  nob_log(INFO, "Creating bundled header...");

  const char *outPath = DIST_DIR "/pdphyzx.h";

  // Create dist directory if needed
  if (!nob_mkdir_if_not_exists(DIST_DIR) || !createGitignore(DIST_DIR)) {
    return false;
  }

  // Step 1: Find and parse headers
  HeaderFile *headers = NULL;
  size_t headerCount = 0;
  if (!findAndParseHeaders(&headers, &headerCount)) {
    return false;
  }

  // Step 2: Topologically sort headers
  const char **sortedFiles = NULL;
  size_t sortedCount = 0;
  if (!topologicallySortHeaders(headers, headerCount, &sortedFiles,
                                &sortedCount)) {
    return false;
  }

  // Step 3: Collect standard library includes
  char *stdIncludes[100] = {0};
  size_t includeCount = 0;
  if (!collectStandardIncludes(stdIncludes, &includeCount)) {
    return false;
  }

  // Step 4: Build output content
  String_Builder output = {0};
  if (!buildOutputContent(&output, sortedFiles, sortedCount, stdIncludes,
                          includeCount)) {
    return false;
  }

  // Step 5: Write output to file
  if (!write_entire_file(outPath, output.items, output.count)) {
    nob_log(ERROR, "Failed to write output file %s", outPath);
    return false;
  }

  nob_log(INFO, "Created bundled header: %s", outPath);

  return true;
}

bool run(Options options) {
  // Execute the PDX file
  const char *pdxPath =
      temp_sprintf("%s/pdphyzx_%s.pdx", BUILD_DIR, options.example);

  // FIXME: This is a hack - figure out why simulator dies on restart --
  Cmd killCmd = {0};
  cmd_append(&killCmd, "killall", "-q", "Playdate Simulator");
  cmd_run_sync(killCmd);
  usleep(5000);
  // --

  Cmd runCmd = {0};
  cmd_append(&runCmd, "open", "-g", pdxPath);
  return cmd_run_sync(runCmd);
}

//==============================================================================
// Help
//==============================================================================

bool usage(const char *program) {
  printf("\n");
  printf("Usage: %s [<command>] [<options>] [<example>]\n", program);

  printf("Commands:\n");
  printf("    all        Build everything (default)\n");
  printf("    build      Build example (all examples if none specified)\n");
  printf("    run        Build and run example\n");
  printf("    bundle     Create single-header distribution file\n");
  printf("    test       Build and run test.c\n");
  printf("    help       Print this message\n");
  printf("\n");

  printf("Options (can be provided in any order):\n");
  printf("    --dev, -d        Build with debug symbols (default)\n");
  printf("    --release, -r    Build optimized release version\n");
  printf("    --target=simulator, -t=simulator, -t=sim\n");
  printf("                     Build for Playdate Simulator (default)\n");
  printf("    --target=device, -t=device, -t=dev\n");
  printf("                     Build for Playdate device (requires "
         "arm-none-eabi-gcc)\n");
  printf("\n");

  printf("Example:\n");
  printf("    Specify an example folder name from ./examples/ directory\n");
  printf("    If not provided, all examples will be built\n");
  printf("\n");
  printf("Examples:\n");
  printf("    %s build sprites    Build the example in examples/sprites\n",
         program);
  printf("    %s run sprites      Build and run example sprites in simulator\n",
         program);
  printf("    %s build            Build all examples\n", program);
  printf("    %s bundle           Create the single-header distribution file\n",
         program);
  printf("\n");

  return true;
}

//==============================================================================
// Main
//==============================================================================

int main(int argc, char **argv) {
  NOB_GO_REBUILD_URSELF(argc, argv);

  const char *programStr = shift(argv, argc);
  Options options = parseBuildOptions(argc, argv);
  bool result = false;

  // Common prep tasks
  switch (options.command) {
  case HELP:
    break;
  case BUNDLE:
    result = setPlaydateSDK() && genLspInfo();
    break;
  default:
    result = setPlaydateSDK() && genLspInfo();
    result = result && (options.skipBundle ? true : bundleHeader());

    break;
  }

  if (!result) {
    usage(programStr);
    return result ? 0 : 1;
  }

  // Actual command execution
  switch (options.command) {
  case ALL:
    result = build(options);
    break;
    // test();
  case BUILD:
    result = build(options);
    break;
  case RUN:
    result = build(options) && run(options);
    break;
  case TEST:
    // test();
    break;
  case BUNDLE:
    result = bundleHeader();
    break;
  case HELP:
    result = usage(programStr);
    break;
  case UNKNOWN:
    result = false;
    break;
  }

  if (!result) {
    usage(programStr);
  }

  return result ? 0 : 1;
}
