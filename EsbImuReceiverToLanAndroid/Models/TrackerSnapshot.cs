using System.Numerics;

namespace EsbReceiverToLanAndroid.Models;

public class TrackerSnapshot
{
    public List<DongleGroup> Dongles { get; set; } = new();
}

public class DongleGroup
{
    public string DeviceKey { get; set; } = "";
    public string DisplayName { get; set; } = "";
    public List<TrackerInfo> Trackers { get; set; } = new();
}

public class TrackerInfo
{
    public int Id { get; set; }
    public string Name { get; set; } = "";
    public string DisplayName { get; set; } = "";
    public float BatteryLevel { get; set; }
    public float BatteryVoltage { get; set; }
    public float? Temperature { get; set; }
    public int SignalStrength { get; set; }
    public string Status { get; set; } = "";
    public Quaternion Rotation { get; set; }
}
