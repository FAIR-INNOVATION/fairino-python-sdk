===================================================================
V2.0.8-robot-v3.7.8 2024.01.20 简介：Python SDK V2.0.8版本，适配法奥机器人V3.7.8版本。
	1.由本版本开始维护此版本更新表；
	2.在版本更新表中记录版本更新过程中的新增、修改或删除的SDK变更。
第三方库依赖关系：
	fr-robot - V3.7.8
===================================================================
V2.0.9-robot-v3.7.9 2024.02.06 简介：Python SDK V2.0.9版本，适配法奥机器人V3.7.9版本。
	1.参数变化
		（1）电弧跟踪控制SDK接口(ArcWeldTraceControl)新增偏置跟踪类型(offsetType)、偏置参数(offsetParameter)参数；
		（2）力传感器辅助拖动SDK接口(EndForceDragControl)新增奇异点策略(ingularityConstraintsFlag)参数。
	2.新增接口
		（1）摆动渐变开始SDK接口(WeaveChangeStart)；
		（2）摆动渐变结束SDK接口(WeaveChangeEnd)。
第三方库依赖关系：
	fr-robot - V3.7.9