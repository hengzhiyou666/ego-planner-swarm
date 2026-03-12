#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${WORKSPACE_DIR}/install"
THIRD_PARTY_DIR="${INSTALL_DIR}/third_party_libs"

# 需要排除的“系统/运行时基础库”前缀（不会被视为需要打包的第三方库）
EXCLUDE_LIB_PREFIXES=(
  "linux-vdso.so"
  "ld-linux"
  "libc.so"
  "libm.so"
  "libpthread.so"
  "librt.so"
  "libdl.so"
  "libgcc_s.so"
  "libstdc++.so"
)

# 需要排除的路径前缀（例如纯系统路径，可按需调整）
#EXCLUDE_PATH_PREFIXES=(
#  "/lib"
#  "/usr/lib"
#)


echo "=== colcon build 开始 ==="
cd "${WORKSPACE_DIR}"

# 允许通过参数把额外参数传给 colcon build
colcon build "$@"

echo "=== colcon build 完成 ==="
echo "安装目录: ${INSTALL_DIR}"

if [[ ! -d "${INSTALL_DIR}" ]]; then
  echo "未找到 install 目录: ${INSTALL_DIR}，退出。" >&2
  exit 1
fi

mkdir -p "${THIRD_PARTY_DIR}"

declare -A COPIED_LIBS=()

echo "=== 扫描 install 中可执行文件和共享库，解析依赖 ==="

# 查找 install 下的所有 ELF 可执行文件和共享库
while IFS= read -r target; do
  # 跳过空行
  [[ -z "${target}" ]] && continue

  # 只处理普通文件
  [[ -f "${target}" ]] || continue

  # 使用 file 判断是否为 ELF，如果不是就跳过
  if ! file "${target}" 2>/dev/null | grep -q "ELF"; then
    continue
  fi

  echo "解析依赖: ${target}"

  # 从 ldd 输出中抽取依赖库的绝对路径
  # 典型行格式: libopencv_core.so.4.5 => /usr/lib/libopencv_core.so.4.5 (0x...)
  # 也有可能是: /lib64/ld-linux-x86-64.so.2 (0x...)
  while IFS= read -r dep; do
    # 过滤掉空行
    [[ -z "${dep}" ]] && continue

    dep_path=""

    # 先处理带 "=>" 的情况
    if grep -q "=>" <<< "${dep}"; then
      dep_path="$(awk '{for(i=1;i<=NF;i++){if($i ~ /^\//){print $i; exit}}}' <<< "${dep}")"
    else
      # 处理没有 "=>" 但直接给出绝对路径的情况
      dep_path="$(awk '{for(i=1;i<=NF;i++){if($i ~ /^\//){print $i; exit}}}' <<< "${dep}")"
    fi

    [[ -z "${dep_path}" ]] && continue
    [[ ! -f "${dep_path}" ]] && continue

    lib_name="$(basename "${dep_path}")"

    # 1) 排除“名称上”就是一些系统基础库的
    skip=false
    for ex in "${EXCLUDE_LIB_PREFIXES[@]}"; do
      if [[ "${lib_name}" == ${ex}* ]]; then
        skip=true
        break
      fi
    done
    [[ "${skip}" == true ]] && continue

    # 2) 排除位于纯系统目录下的（/lib /usr/lib 等），避免把所有系统库都拷贝进来
    for pfx in "${EXCLUDE_PATH_PREFIXES[@]}"; do
      if [[ "${dep_path}" == ${pfx}/* ]]; then
        skip=true
        break
      fi
    done
    [[ "${skip}" == true ]] && continue

    # 剩下的认为是“需要随程序一起打包”的第三方库
    COPIED_LIBS["${dep_path}"]=1

  done < <(ldd "${target}" 2>/dev/null | sed 's/^[[:space:]]*//')

done < <(find "${INSTALL_DIR}" \( -type f -executable -o -name "*.so" -o -name "*.so.*" \) 2>/dev/null)

echo "=== 开始拷贝解析到的第三方库到: ${THIRD_PARTY_DIR} ==="

if [[ ${#COPIED_LIBS[@]} -eq 0 ]]; then
  echo "未解析到需要拷贝的第三方库。"
  echo "如需调整判定规则，可修改 EXCLUDE_LIB_PREFIXES / EXCLUDE_PATH_PREFIXES。"
  exit 0
fi

for src in "${!COPIED_LIBS[@]}"; do
  base="$(basename "${src}")"
  dst="${THIRD_PARTY_DIR}/${base}"

  # 不重复拷贝完全相同的文件
  if [[ -f "${dst}" ]]; then
    continue
  fi

  # 若依赖路径本身是符号链接，则解析到实际文件再拷贝
  real_src="${src}"
  if [[ -L "${src}" ]]; then
    real_src="$(readlink -f "${src}" || echo "${src}")"
    echo "解析符号链接: ${src} -> ${real_src}"
  fi

  echo "拷贝: ${real_src} -> ${dst}"
  # 使用 -L 选项，确保即使 real_src 仍是链接也会拷贝实际文件内容
  cp -L -p "${real_src}" "${dst}"
done

echo "=== 第三方库拷贝完成 ==="
echo "已拷贝到目录: ${THIRD_PARTY_DIR}"

echo "提示: 请确保运行时设置合适的 LD_LIBRARY_PATH，例如："
echo "  export LD_LIBRARY_PATH=\"${THIRD_PARTY_DIR}:\${LD_LIBRARY_PATH:-}\""

