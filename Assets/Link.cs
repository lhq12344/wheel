using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArticulationArmBindToCar : MonoBehaviour
{
    [Header("Car install point")]
    public Transform carMount;                  // CarRoot/ArmMount

    [Header("Arm root articulation body (base link)")]
    public ArticulationBody armRoot;            // Link_00 上的 ArticulationBody（根）

    [Header("Options")]
    [Tooltip("Bind in FixedUpdate for physics consistency")]
    public bool bindInFixedUpdate = true;

    [Tooltip("Disable collision between arm and chassis by layers is recommended; this is only a reminder flag")]
    public bool recommendedDisableArmChassisCollision = true;

    void Reset()
    {
        // 尝试自动找一个 articulation root（不保证正确，建议手动拖拽确认）
        armRoot = GetComponentInChildren<ArticulationBody>();
    }

    void Awake()
    {
        if (armRoot == null)
        {
            Debug.LogError("[ArmBind] armRoot is null. Please assign the root ArticulationBody (usually Link_00).");
        }
        if (carMount == null)
        {
            Debug.LogError("[ArmBind] carMount is null. Please assign CarRoot/ArmMount transform.");
        }
    }

    void FixedUpdate()
    {
        if (!bindInFixedUpdate) return;
        DoBind();
    }

    void LateUpdate()
    {
        if (bindInFixedUpdate) return;
        // 如果你坚持不用物理步绑定，可以用 LateUpdate，但稳定性不如 FixedUpdate
        DoBind();
    }

    void DoBind()
    {
        if (armRoot == null || carMount == null) return;

        // 关键：TeleportRoot 会把整条 articulation 链整体搬运到目标位姿
        armRoot.TeleportRoot(carMount.position, carMount.rotation);

        // 保险：某些情况下同步一次 transforms 有助于避免视觉延迟
        //（如果你观察到画面一帧不同步，可以保留）
        Physics.SyncTransforms();
    }
}
