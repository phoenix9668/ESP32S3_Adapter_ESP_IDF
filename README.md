# ESP32-S3 Adapter V2

本工程是 ESP32-S3 Adapter V2 的单固件实现。ESP32-S3 直接管理 ML307C
AT、OneNET MQTT、GNSS、RFID 持久化队列和 OneNET 直连 OTA；不再依赖
EC800K/QuecPython，也不需要 VPS 或自有域名。

## 固定工具链

工程只允许 ESP-IDF v5.5.4 和 `esp32s3` target。CMake 会检查精确版本，
用 v6.x 或其他版本配置会立即失败。

首次安装：

```sh
/Users/gally/.espressif/v5.5.4/esp-idf/install.sh esp32s3
```

每个新终端初始化：

```sh
export IDF_PATH=/Users/gally/.espressif/v5.5.4/esp-idf
export IDF_TOOLS_PATH=/Users/gally/.espressif
source /Users/gally/.espressif/v5.5.4/esp-idf/export.sh
idf.py --version
idf.py set-target esp32s3
idf.py fullclean build
```

`idf.py --version` 必须输出 `ESP-IDF v5.5.4`。工具实际位于
`/Users/gally/.espressif/tools`，但 ESP-IDF 的 `IDF_TOOLS_PATH` 定义的是其
父目录，所以变量值必须为 `/Users/gally/.espressif`。VS Code 已固定相同路径，
控制台走 USB Serial/JTAG；UART0 因而可专用于 ML307C。

## ML307C 安全接线

- LLMM307R 核心板使用外部 12 V 接 VIN，并与 ESP32-S3 共地。
- GPIO43（ESP TX）连接核心板 RX。
- GPIO44（ESP RX）连接核心板 TX。
- H2.3/U2RXD 到 BAT 必须断开。
- H2.4/U2TXD 到 EN 必须断开。
- H2 的 5 V 不连接。

固件使用 UART0、115200、8N1。GNSS 上电后保持开启并持续搜星，每 120 秒查询
一次定位。固件强制保持 `nmea/mask=0` 和 `MGNSSLOC=0`，只关闭 AT 串口上的
NMEA/位置自动上报，不关闭 GNSS 引擎；这样既保留热定位状态，又避免 NMEA URC
插入 HTTPS HEX 数据帧。固件不驱动 BAT/EN，也不执行 GPIO 断电或硬复位。

## Device Key、时间与动态 Token

新设备 NVS namespace 为 `onenet`：

- `broker_host`、`broker_port`
- `product_id`、`device_name`
- `device_key`：每台设备自己的 OneNET Device Key
- `token_ttl`：动态 Token 有效期，默认 3600 秒

ML307C 完成网络注册后，固件先用 `AT+CCLK?` 获取运营商时间并校验，然后在
每次 MQTT 重连时以 Device Key 动态生成 MQTT HMAC-SHA256 Token。OneNET OTA
Authorization 使用相同 Device Key 生成一小时有效的 HMAC-SHA1 设备级 Token，
鉴权资源固定为 `products/{productId}/devices/{deviceName}`。日志不会打印
Device Key、完整 Token 或 Authorization。

旧 NVS 的 `access_key`、`token_expiry` 仍可兼容读取，但新设备不要继续写入
绝对过期时间。单台开发板可使用：

```sh
cp tools/onenet_nvs.csv.example tools/onenet_nvs.csv
# 本地填写后仅生成 NVS：
bash tools/provision_onenet.sh
# 或直接烧写 NVS：
bash tools/provision_onenet.sh /dev/tty.usbmodem11201
```

本地 CSV 和生成的 NVS bin 已加入 Git 忽略规则。

## OneNET 物模型与固件版本属性

OTA 使用独立的 `/fuse-ota/.../version` 接口；为了同时在设备“属性”页面直观
查看当前运行版本，产品物模型还需要包含以下自定义属性：

| 配置项 | 值 |
|---|---|
| 功能名称 | 固件版本 |
| 标识符 | `firmware_version` |
| 数据类型 | `string` |
| 数据长度 | `32` |
| 读写类型 | 只读（`r`，设备可上报，云端只展示/查询） |
| 是否必选 | 否 |

在 OneNET 控制台进入“产品开发 → wireless-module → 功能定义”，添加并发布这个
属性。仓库中的 `doc/model-yCwI3MB0Kq.json` 已同步加入相同定义，可用于核对或
重新导入物模型。未在平台发布该标识符时，OneNET 会拒绝属性上报，但不会影响
OTA 的独立版本接口。

固件每次 MQTT 连接成功后，从应用镜像描述中读取版本并上报；该版本由根目录
`version.txt` 在构建时写入。只有匹配请求 ID 且回复 `code=200` 才记为成功，
失败按一分钟间隔重试，不阻塞 RFID 本地采集。成功日志示例：

```text
I (...) CELLULAR: firmware_version=1.0.0 delivered
```

随后进入“设备接入管理 → 设备管理 → wireless-module-001 → 属性”，即可查看
`firmware_version` 的最新值。OTA 页面中的设备版本仍由 OTA 专用接口维护，
两种上报使用同一个镜像版本号，但彼此独立。

## 100 台量产固化

量产工具入口是 `tools/fleet`。公共签名固件只构建一次，每台设备只临时生成
一个专属 NVS。加密清单默认保存在 `.factory/fleet.enc`，采用 scrypt 派生密钥
和 AES-256-GCM 加密，文件权限为 0600。

### 1. 准备量产电脑

复制公共配置：

```sh
cp tools/fleet_config.json.example tools/fleet_config.json
```

填写产品 ID、MQTT 域名、端口和设备名前缀。`tools/fleet_config.json` 已被
Git 忽略。当前产品可使用页面显示的 `yCwI3MB0Kq` 和
`yCwI3MB0Kq.mqtts.acc.cmcconenet.cn`；端口以 OneNET 产品页面实际配置为准。

在当前终端设置以下环境变量：

```sh
export ONENET_USER_ID='你的 OneNET 用户 ID'
export ONENET_API_ACCESS_KEY='账号 OpenAPI 的 Base64 Access Key'
export ONENET_PRODUCT_ID='产品 ID'
export FACTORY_MANIFEST_PASSPHRASE='至少 12 位的量产清单密码'
```

这里的 `ONENET_API_ACCESS_KEY` 只供量产电脑调用 OpenAPI，不是产品
access_key，也不是设备 Device Key。不要把这些 export 写进仓库脚本。密码丢失
后无法恢复加密清单；应将密码和 `.factory/fleet.enc` 分开离线备份。

### 2. 导入 001 并创建 002–100

```sh
tools/fleet prepare --start 1 --count 100
tools/fleet status
```

工具逐个检查 `wireless-module-001` 到 `wireless-module-100`：

- 已存在的 001 会从 OneNET 查询并保存当前已重置的 Device Key，不会重建。
- 缺失设备通过 OneNET `BatchCreateDevices` 一次批量创建；OneNET 单次上限为
  500，因此 100 台无需拆批。
- 每台返回的 Device Key 立即写入加密清单，终端不显示密钥。
- 已安全保存到本地的身份重复执行 `prepare` 时不会再次创建。

准备后立即备份 `.factory/fleet.enc`。不要导出明文设备密钥表。

### 3. 生成并备份 RSA 签名密钥

只执行一次：

```sh
tools/generate_signing_key.sh
```

默认生成的 `keys/ota_signing_key.pem` 是 RSA-3072 私钥并已被 Git 忽略。正式量产
建议把加密移动介质挂载后执行
`OTA_SIGNING_KEY=/Volumes/OFFLINE/ota_signing_key.pem tools/generate_signing_key.sh`；
工具只在工程内创建一个被忽略的符号链接。将私钥备份到至少两个离线加密介质，
再开始量产。已出货设备只接受该密钥签名的 OTA 应用；
私钥丢失将永久失去后续 OTA 发布能力，替换私钥不能修复已出货设备。

本方案启用签名应用校验和 OTA 回滚，但暂不烧录不可逆 Secure Boot eFuse，
也不启用 Flash Encryption。

### 4. 公共 factory 固件只构建一次

```sh
export IDF_PATH=/Users/gally/.espressif/v5.5.4/esp-idf
export IDF_TOOLS_PATH=/Users/gally/.espressif
source /Users/gally/.espressif/v5.5.4/esp-idf/export.sh
idf.py fullclean build
```

必须确认构建日志显示 v5.5.4、target `esp32s3`，并确认 `ota_0`/`ota_1`
各为 4 MiB。量产期间不要重新 `fullclean`，`fleet station` 会复用 `build` 中的
bootloader、分区表、初始 otadata 和签名应用。

### 5. 连续固化 100 台

量产电脑一次只连接一台 ESP32-S3：

```sh
tools/fleet station --port auto
```

每台流程为：

1. 自动选择唯一 USB 串口并读取 ESP32-S3 MAC。
2. 如果 MAC 曾经分配过身份，则复用原设备名；否则分配最小未使用设备名。
3. 在任何烧录发生前，先持久化 MAC→设备名绑定。
4. 临时生成专属 NVS。
5. 一次写入 bootloader、分区表、初始 otadata、签名应用和专属 NVS；不会擦除
   或写入 `rfid_store`。
6. 复位后依次等待 `nvs`、`modem`、`sim` 和 `online` 工厂状态。
7. MQTT 成功连接 OneNET 后记录 ICCID、版本和时间，显示 `PASS`。
8. 临时明文 CSV/NVS 自动删除；操作员贴设备名/MAC 标签并更换下一台。

自动端口要求电脑上只有一个候选 USB 串口。否则明确指定：

```sh
tools/fleet station --port /dev/tty.usbmodem11201
```

### 6. 失败、返修和审计

烧录或联网失败后停止更换设备，保留现场并执行：

```sh
tools/fleet retry --mac AA:BB:CC:DD:EE:FF --port auto
```

`retry` 只允许已分配的 MAC，并强制复用原身份。即使第一次烧录断电，也不会
占用下一个设备名。外部工具误写了 NVS 时，同样用真实 MAC 执行 `retry` 恢复
清单中绑定的身份。

- MAC 已存在：视为返修件，复用原身份，不新增设备。
- 同一设备名已绑定其他 MAC：工具拒绝烧录，隔离两台实物后核对标签。
- OneNET Device Key 被重置：不要手工复制旧 NVS；先在安全清单中更新身份，
  再返修烧写：

```sh
tools/fleet refresh-key --device-name wireless-module-001 --confirm
tools/fleet retry --mac AA:BB:CC:DD:EE:FF --port auto
```

`refresh-key` 会从 OneNET 重新查询密钥并加密保存，但不显示密钥；没有显式
`--confirm` 时拒绝修改。
- 无法确认身份的板卡：隔离，不要通过修改 CSV 抢占新设备名。

查看进度和导出不含密钥的审计表：

```sh
tools/fleet status
tools/fleet export-audit --output .factory/fleet-audit.csv
```

审计 CSV 仅包含设备名、MAC、ICCID、版本、分配/烧录/验证时间和结果。量产结束
后确认 `.factory` 中不存在 `.nvs.csv`/`.nvs.bin`，离线备份加密清单，再从量产
电脑安全移除临时文件。

## OneNET 直连 OTA

设备直接访问 `https://iot-api.heclouds.com/fuse-ota/...`，不需要 VPS、域名或
对象存储。实现包含：版本上报、任务检查、任务状态检查、HTTP Range 下载、
状态上报、MQTT 通知和六小时主动轮询。

MQTT 订阅和回复：

```text
$sys/{productId}/{deviceName}/ota/inform
$sys/{productId}/{deviceName}/ota/inform_reply
```

通知使用原始请求 ID 回复 `code=200`。启动、MQTT 重连及每六小时都会主动检查，
因此 QoS 0 通知丢失不会永久漏掉任务。

只接受 type=2 的 SOTA/MCU 应用完整包、严格高于当前版本的目标版本、大小不超过
非活动 4 MiB OTA 分区且 ESP 镜像 project name 为本工程的任务。下载期间暂停
GNSS 查询和普通 MQTT 上传，但 E34/RFID 采集任务继续运行，RFID 仍写持久化
FIFO。

### 1. 构建唯一上传文件

先修改并提交 `version.txt`，例如从 `1.0.0` 改为 `1.1.0`，然后执行（参数与
`version.txt` 不一致时脚本会拒绝发布）：

```sh
tools/build_ota.sh 1.1.0
```

脚本会强制 ESP-IDF v5.5.4、执行 clean build、使用 RSA-3072 签名、再次验签并
生成：

```text
dist/s3-1.1.0.bin
dist/s3-1.1.0.manifest.json
```

只把第一个 `.bin` 上传 OneNET。manifest 留在本地用于核对版本、大小、MD5、
SHA-256、Git commit 和构建时间。

严禁上传 merged factory image、`bootloader.bin`、分区表、`ota_data_initial.bin`
或 NVS。OneNET 上的一个 `.bin` 是完整的 ESP-IDF 应用镜像，作用等同于旧
QuecPython 的单文件 `main.py.bin`，但它只写非活动 OTA 应用分区。

### 2. 在 OneNET 创建升级包

进入 OneNET 控制台的“增值服务 → 远程升级”，添加升级包：

- 产品：`wireless-module`
- 升级模块：MCU 软件/应用软件
- 类型：SOTA 完整包
- 目标版本：必须与脚本参数和镜像内版本完全一致，例如 `1.1.0`
- 文件：`s3-1.1.0.bin`

OneNET 要求上传文件名为 1–20 个英文字母、数字、点、连字符或下划线。
发布脚本使用短文件名 `s3-{version}.bin` 并在构建前检查长度；不要把本地
`.manifest.json`、merged factory image、bootloader 或分区表上传为升级包。

先只选择 `wireless-module-001` 做验证升级，通知方式选择 MQTT。设备应依次出现：
版本上报、任务检查、分片下载进度、100%、重启、新版本运行、最终成功。

001 稳定验证后，剩余设备按 `1 → 9 → 30 → 60` 四批发布。每批都要观察成功率
和回滚记录，再开始下一批；不要直接勾选全部 100 台。

### 3. 断点续传、验签和回滚

- 每个 HTTP Range 最大 256 KiB，实际每次最多 4 KiB 写入非活动 OTA 分区；
  下载开始和完成各上报一次状态，避免每个持久化检查点都重新 TLS 建连。
- ML307C 在固定 115200 波特率下把二进制 HTTP 响应转成两倍长度的 HEX 文本，
  因而 UART 侧净载荷理论上限约 5.7 KiB/s。约 725 KiB 的应用在网络稳定时通常
  需要 2–3 分钟；若耗时明显更长，应检查日志中是否出现 `content gap`、重连和
  退避。驱动会按 `cur_len` 统计 HEX 字节，并忽略模组在数据内部插入的 CRLF
  以及偶发的单独 CR/LF；其他非法字节会记录帧内位置并触发安全续传。
- 同一个 Range 连接内每完成 16 KiB，就在 NVS `ota` namespace 只更新持久化
  偏移；任务 ID、版本、MD5 和分区只在任务开始/状态切换时写入。断网或重启后
  从 4 KiB 擦除边界恢复，并在重写前擦除检查点之后的 Flash 扇区，避免断流
  数据污染 MD5，同时把单次断流需要重下的数据限制在 16 KiB 左右。
- HTTP 超时或断网只暂停下载并保留检查点，不会上报会结束云端任务的终态；
  恢复网络后按 2、5、15、30、60 秒退避继续同一任务，后续重试封顶 60 秒。
- 下载完成依次校验总长度、OneNET MD5、ESP 镜像 project/version 和 RSA-3072
  应用签名。MD5 不一致会清除坏检查点并自动完整重下 1 次；第二次仍不一致才
  上报 `205`。任一失败都不会切换启动分区。
- 新镜像启动后进入 rollback trial。NVS、RFID Store、主要任务和 `onenet`
  配置成功启动并稳定 60 秒后才标记有效；蜂窝网络是否在线不作为回滚条件。
- 新版本先完成本地 60 秒试运行，期间暂不上报新版本；标记有效后先补报
  `step=201`，再上报新版本。`step=100` 已把平台任务切换到“升级中”，因此不再
  重复上报会被平台拒绝的 `step=101`。若新固件崩溃或看门狗复位，bootloader
  自动回滚，旧版本恢复联网后补报失败。
- OneNET 明确返回终态上报已完成、已取消或状态无效时，设备会清理该任务上下文；
  网络/HTTP 传输失败才保留并重试，避免旧 `step=201` 阻塞版本上报和后续任务。
- OTA 不修改 NVS 凭据、RFID Store、bootloader 或分区表。

常用终态：`102` OTA 分区空间不足、`204` 镜像版本不一致、`205` MD5 校验失败、
`206` project、RSA/镜像签名或安装失败。临时下载/网络失败会保留为可恢复状态。
日志会输出期望和实际 MD5，但不会输出 Device Key、token 或 Authorization。
出现失败时先检查 OneNET 任务版本、完整包类型、上传文件 MD5、应用大小、SIM
网络和串口日志，不要重置 Device Key。

旧固件已经上报 `205` 且 NVS 中保存了 100% 坏检查点时，不要继续等待原任务。
先通过 USB 烧录包含本节修复的应用（保留 `onenet` 和 `rfid_store` 分区），再在
OneNET 创建一个新任务 ID；新固件也会拒绝把 `offset == size` 当作有效断点。

本工程提供专用恢复脚本，不会改写 OneNET NVS、RFID Store、bootloader 或分区
表。脚本只擦除 8 KiB `otadata`、把签名应用写入 `ota_0`，再由 bootloader 从
`ota_0` 启动。不能只运行普通 `idf.py app-flash`：设备当前可能正从 `ota_1`
启动，单独写 `ota_0` 而不重置 boot selection 不一定会运行新镜像。

先生成签名恢复镜像，再把端口替换成实际枚举出的 USB Serial/JTAG 设备：

```sh
tools/build_ota.sh 1.0.2
tools/recover_ota_app.sh /dev/tty.usbmodemXXXX dist/s3-1.0.2.bin
export IDF_TOOLS_PATH=/Users/gally/.espressif
. /Users/gally/.espressif/v5.5.4/esp-idf/export.sh
idf.py -p /dev/tty.usbmodemXXXX monitor
```

不要执行 `erase-flash`。恢复固件上线并重新上报当前版本后，应删除/停止旧的失败
任务，使用版本更高的新 `.bin`（例如 1.0.3）新建任务验证 OTA 修复。

如果两个应用槽都无法启动，使用 USB Serial/JTAG 进入下载模式，按“公共
factory 固件 + 该 MAC 原身份专属 NVS”返修烧录；不要擦除或改写
`rfid_store`。只有明确接受丢失队列数据时才能整片擦除。

## 数据交付行为

- 每个有效 RFID 标签先写 `rfid_store`，OneNET 属性回复的请求 ID 匹配且
  `code=200` 后才删除，提供至少一次交付。
- GNSS 只保留最新 generation，无定位时上传 `fix_status=0`，不伪造坐标。
- 上传调度优先到期 GNSS，再处理最旧 RFID，并在每条 RFID 后重新检查 GNSS。
- E34 保留原 `0xE55E + 地址/类型 + payload + CRC32` 帧格式，流解析支持拆包、
  粘包、噪声和 CRC 错误重同步。

## 测试

主机协议测试：

```sh
cmake -S tests/host -B /tmp/esp32-adapter-host-tests
cmake --build /tmp/esp32-adapter-host-tests
ctest --test-dir /tmp/esp32-adapter-host-tests --output-on-failure
```

量产清单测试：

```sh
/Users/gally/.espressif/python_env/idf5.5_py3.12_env/bin/python \
  tests/test_fleet.py
```

测试覆盖 E34 流解析、RFID 队列恢复、OneNET MQTT/OTA Token、回复匹配、GNSS
空字段和南/西半球坐标、网络时间、OTA 通知/任务解析、版本降级拒绝，以及加密
清单和 MAC 身份复用。最终发布还必须在 v5.5.4 下执行签名 `fullclean build`，
再在真实 `wireless-module-001` 上完成一次 OneNET OTA 验证。

OneNET 参考：[批量创建设备](https://iot.10086.cn/doc/iot_platform/book/api/common/batchCreateDevice.html)、
[OpenAPI 鉴权](https://iot.10086.cn/doc/iot_platform/book/api/auth.html)、
[检测升级任务](https://iot.10086.cn/doc/aiot/fuse/detail/1447)、
[上报设备版本](https://iot.10086.cn/doc/aiot/fuse/detail/1504)。
