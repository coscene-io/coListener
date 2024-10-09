## VERSION: 1.1.0

* 新增log上传功能
* 修正"规则版本检测" 中的bug 
* code format

## VERSION： 1.0.0

* 从云端获取rules时，优先获取rules version，如果version未发生变化，则不再获取rules，以节省流量。
* 成功获取rules后，存入本地cache，解决网络环境较差时，长时间无法正常初始化规则引擎的问题。
* 正式定版1.0.0，并在上传的record description中添加版本号信息，方便后期在多版本情况下分析定位问题。
* 每小时（3600秒）重新加载一次配置文件，防止因device api key过期导致数据无法上传。