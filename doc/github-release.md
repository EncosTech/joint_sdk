# GitHub 发布与构建产物

两条分支的项目自有代码采用 MIT；第三方子模块保留原许可证。原生 DEB 和 npm tarball 均携带 LICENSE。
DEB 的 `share/doc/jointsdk/copyright` 声明 MIT 并保留 Boost 许可，npm 的 `license` 字段为 MIT。
上传 artifact 前会执行 `scripts/verify-package-license.py` 检查实际包内容，缺失许可证即失败。

`build-native` 每次成功构建都会生成 DEB 并上传 `deb-<系统>-<架构>-<SHA>` artifact；
`build-wasm` 完成测试后打包并上传 `npm-wasm-<SHA>` artifact。下载入口为对应 Actions 运行记录。
Gitea 软件包仓库发布仍只在 stable 推送且版本变更时执行。

从 motor_driver 引入的 `.gitea/workflows/publish-github-release.yml` 支持手动触发：
配置变量 `RELEASE_ALLOWED_ACTORS`（逗号分隔）、`GITEA_USERNAME`、
`GITHUB_RELEASE_REPOSITORY`、`GITHUB_RELEASE_REPOSITORY_URL`、`GITEA_PACKAGE_OWNER`，
以及 secrets `GITEA_TOKEN`、`GH_TOKEN`。`MAIN_BRANCH` 默认 stable；
`OPTIONAL_BRANCHES=feat/zqy/driver_v4` 可同步功能分支快照。
`DEBIAN_DISTRIBUTIONS` 默认 jammy,noble，`DEBIAN_ARCHITECTURES` 默认 amd64,arm64。
发布脚本提取 joint_sdk 版本，仅选择 jointsdk Debian 包，校验大小和 SHA256 后上传 GitHub Release。
可选分支只同步源码快照，使用主发布分支版本号。

本地使用：复制 `scripts/.env.example` 为被 Git 忽略的 `scripts/.env`，填写配置，先执行：

```bash
bash scripts/publish-github-release.sh --dry-run
# 确认配置后实际发布：
bash scripts/publish-github-release.sh
```

`--main-sha1` 可指定 stable 历史提交；不提供时使用分支头。
发布集成测试使用临时本地 Git 仓库与模拟 HTTP，不会触发真实发布：

```bash
bash tests/publish_github_release_wrapper_test.sh
bash tests/release_to_github_test.sh
```

stable 跟随 motor_driver 的 `main`（v3，最低 3.3.2）；native 和 WASM 使用相同分支。
功能分支跟随 driver 的 `feat/zqy/v4`。
