# D:\dream\HF2026\src\vision_detect\main.py
# 视觉识别模块独立测试脚本

import random
import time
from decoy_classifier import DecoyClassifier


def test_real_target():
    """测试真目标（匀速运动）"""
    print("\n=== 测试真目标（匀速运动）===")
    classifier = DecoyClassifier()
    
    for i in range(40):
        # 模拟匀速运动 + 噪声
        lat = 27.0 + i * 0.0002 + random.uniform(-0.00005, 0.00005)
        lon = 125.0 + i * 0.0003 + random.uniform(-0.00005, 0.00005)
        classifier.update(lat, lon, 0.1)
        
        if i % 10 == 0:
            info = classifier.get_debug_info()
            print(f"  第{i}帧: is_real={info['is_real']}, conf={info['confidence']:.2f}, speed={info['speed']:.2f}m/s")
    
    print(f"最终判断: {'真目标 ✅' if classifier.is_real_target else '诱饵 ❌'}")
    return classifier.is_real_target


def test_decoy():
    """测试诱饵（随机游走，模拟噪声驱动）"""
    print("\n=== 测试诱饵（随机游走）===")
    classifier = DecoyClassifier()
    
    for i in range(40):
        # 诱饵在原地随机抖动（模拟噪声）
        lat = 27.0 + random.uniform(-0.0002, 0.0002)
        lon = 125.0 + random.uniform(-0.0002, 0.0002)
        classifier.update(lat, lon, 0.1)
        
        if i % 10 == 0:
            info = classifier.get_debug_info()
            print(f"  第{i}帧: is_real={info['is_real']}, conf={info['confidence']:.2f}, speed={info['speed']:.2f}m/s")
    
    print(f"最终判断: {'真目标 ✅' if classifier.is_real_target else '诱饵 ❌'}")
    return classifier.is_real_target


def test_should_report():
    """测试上报判断逻辑"""
    print("\n=== 测试上报判断 ===")
    classifier = DecoyClassifier()
    
    # 模拟一个真目标
    for i in range(50):
        lat = 27.0 + i * 0.0002 + random.uniform(-0.00003, 0.00003)
        lon = 125.0 + i * 0.0003 + random.uniform(-0.00003, 0.00003)
        classifier.update(lat, lon, 0.1)
        
        if classifier.should_report():
            pos = classifier.get_report_position()
            print(f"  ✅ 第{i}帧: 应该上报! 位置: ({pos[0]:.6f}, {pos[1]:.6f})")
            classifier.reset()  # 上报后重置


if __name__ == "__main__":
    print("=" * 50)
    print("视觉识别模块独立测试")
    print("=" * 50)
    
    # 运行测试
    result1 = test_real_target()
    result2 = test_decoy()
    
    print("\n" + "=" * 50)
    print("测试总结:")
    print(f"  真目标测试: {'通过 ✅' if result1 else '失败 ❌'}")
    print(f"  诱饵测试: {'通过 ✅' if not result2 else '失败 ❌'}")
    
    # 运行上报测试
    test_should_report()
    
    print("\n所有测试完成！")