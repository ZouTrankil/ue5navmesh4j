package odin.recast.algorithms;

import odin.recast.config.RecastEnums.LogCategory;
import odin.recast.config.RecastEnums.TimerLabel;

/**
 * Recast构建上下文
 * 翻译自UE5 rcContext类
 * 
 * 提供日志记录和性能计时功能
 * 
 * @author UE5NavMesh4J
 */
public class RecastContext {
    
    /** 是否启用日志记录 */
    private boolean logEnabled;
    
    /** 是否启用计时器 */
    private boolean timerEnabled;
    
    /** 计时器开始时间 */
    private long[] timerStart;
    
    /** 累计时间 */
    private long[] accumulatedTime;
    
    /**
     * 默认构造函数
     */
    public RecastContext() {
        this(true);
    }
    
    /**
     * 构造函数
     * @param enableLog 是否启用日志
     */
    public RecastContext(boolean enableLog) {
        this.logEnabled = enableLog;
        this.timerEnabled = true;
        
        // 初始化计时器数组
        int maxTimers = TimerLabel.RC_MAX_TIMERS.ordinal();
        this.timerStart = new long[maxTimers];
        this.accumulatedTime = new long[maxTimers];
        
        resetTimers();
    }
    
    /**
     * 设置日志启用状态
     * @param enabled 是否启用
     */
    public void enableLog(boolean enabled) {
        this.logEnabled = enabled;
    }
    
    /**
     * 检查日志是否启用
     * @return true如果启用
     */
    public boolean isLogEnabled() {
        return logEnabled;
    }
    
    /**
     * 重置日志
     */
    public void resetLog() {
        if (logEnabled) {
            doResetLog();
        }
    }
    
    /**
     * 记录日志
     * @param category 日志分类
     * @param format 格式字符串
     * @param args 参数
     */
    public void log(LogCategory category, String format, Object... args) {
        if (!logEnabled) {
            return;
        }
        
        String message = String.format(format, args);
        doLog(category, message, message.length());
    }
    
    /**
     * 记录日志消息
     * @param category 日志类别
     * @param message 日志消息
     */
    public void log(LogCategory category, String message) {
        if (this.logEnabled) {
            String prefix = "";
            switch (category) {
                case RC_LOG_PROGRESS:
                    prefix = "[PROGRESS] ";
                    break;
                case RC_LOG_WARNING:
                    prefix = "[WARNING] ";
                    break;
                case RC_LOG_ERROR:
                    prefix = "[ERROR] ";
                    break;
            }
            System.out.println(prefix + message);
        }
    }
    
    /**
     * 设置计时器启用状态
     * @param enabled 是否启用
     */
    public void enableTimer(boolean enabled) {
        this.timerEnabled = enabled;
    }
    
    /**
     * 检查计时器是否启用
     * @return true如果启用
     */
    public boolean isTimerEnabled() {
        return timerEnabled;
    }
    
    /**
     * 重置计时器
     */
    public void resetTimers() {
        if (timerEnabled) {
            doResetTimers();
        }
    }
    
    /**
     * 开始计时
     * @param label 计时器标签
     */
    public void startTimer(TimerLabel label) {
        if (timerEnabled) {
            doStartTimer(label);
        }
    }
    
    /**
     * 停止计时
     * @param label 计时器标签
     */
    public void stopTimer(TimerLabel label) {
        if (timerEnabled) {
            doStopTimer(label);
        }
    }
    
    /**
     * 获取累计时间
     * @param label 计时器标签
     * @return 累计时间（毫秒），-1表示计时器未启用
     */
    public long getAccumulatedTime(TimerLabel label) {
        return timerEnabled ? doGetAccumulatedTime(label) : -1;
    }
    
    // 受保护的方法，子类可以重写以提供具体实现
    
    /**
     * 具体的日志实现
     * @param category 日志分类
     * @param message 消息
     * @param length 消息长度
     */
    protected void doLog(LogCategory category, String message, int length) {
        // 默认实现：输出到控制台
        String categoryStr;
        switch (category) {
            case RC_LOG_PROGRESS:
                categoryStr = "[PROGRESS]";
                break;
            case RC_LOG_WARNING:
                categoryStr = "[WARNING]";
                break;
            case RC_LOG_ERROR:
                categoryStr = "[ERROR]";
                break;
            default:
                categoryStr = "[INFO]";
                break;
        }
        
        System.out.println(categoryStr + " " + message);
    }
    
    /**
     * 重置日志的具体实现
     */
    protected void doResetLog() {
        // 默认实现：无操作
    }
    
    /**
     * 重置计时器的具体实现
     */
    protected void doResetTimers() {
        for (int i = 0; i < timerStart.length; i++) {
            timerStart[i] = 0;
            accumulatedTime[i] = 0;
        }
    }
    
    /**
     * 开始计时的具体实现
     * @param label 计时器标签
     */
    protected void doStartTimer(TimerLabel label) {
        int index = label.ordinal();
        if (index >= 0 && index < timerStart.length) {
            timerStart[index] = System.currentTimeMillis();
        }
    }
    
    /**
     * 停止计时的具体实现
     * @param label 计时器标签
     */
    protected void doStopTimer(TimerLabel label) {
        int index = label.ordinal();
        if (index >= 0 && index < timerStart.length) {
            long endTime = System.currentTimeMillis();
            long elapsed = endTime - timerStart[index];
            accumulatedTime[index] += elapsed;
        }
    }
    
    /**
     * 获取累计时间的具体实现
     * @param label 计时器标签
     * @return 累计时间（毫秒）
     */
    protected long doGetAccumulatedTime(TimerLabel label) {
        int index = label.ordinal();
        if (index >= 0 && index < accumulatedTime.length) {
            return accumulatedTime[index];
        }
        return -1;
    }
    
    /**
     * 获取所有计时器的统计信息
     * @return 计时器统计字符串
     */
    public String getTimerStats() {
        if (!timerEnabled) {
            return "计时器未启用";
        }
        
        StringBuilder sb = new StringBuilder();
        sb.append("Recast 性能统计:\n");
        
        for (TimerLabel label : TimerLabel.values()) {
            if (label == TimerLabel.RC_MAX_TIMERS) {
                continue;
            }
            
            long time = doGetAccumulatedTime(label);
            if (time > 0) {
                sb.append(String.format("  %s: %d ms\n", label.name(), time));
            }
        }
        
        return sb.toString();
    }
    
    /**
     * 打印计时器统计信息
     */
    public void printTimerStats() {
        System.out.println(getTimerStats());
    }
} 