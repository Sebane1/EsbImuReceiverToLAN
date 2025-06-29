using System;
using System.Collections.Generic;
using System.IO;
using HidSharp;

public class HidBufferedReader {
    private readonly HidStream stream;
    private readonly int reportSize;
    private readonly byte[] tempBuffer;
    private readonly MemoryStream buffer;

    public HidBufferedReader(HidStream stream) {
        this.stream = stream;
        this.reportSize = stream.Device.GetMaxInputReportLength();
        this.tempBuffer = new byte[reportSize];
        this.buffer = new MemoryStream();
    }

    /// <summary>
    /// Reads as many HID reports as are available and returns the combined buffer.
    /// Mimics Kotlin's readAll(), aggregating all data currently readable.
    /// </summary>
    public byte[] ReadAll(int timeoutMs = 5, int maxReports = 16) {
        buffer.SetLength(0); // Clear buffer
        stream.ReadTimeout = timeoutMs;

        for (int i = 0; i < maxReports; i++) {
            try {
                int read = stream.Read(tempBuffer, 0, reportSize);
                if (read > 0) {
                    buffer.Write(tempBuffer, 0, read);
                    if (read < reportSize)
                        break; // Partial read, stop here
                } else {
                    break; // Nothing more to read
                }
            } catch (TimeoutException) {
                break; // No more data available
            } catch (IOException) {
                break; // Device disconnected or read error
            }
        }

        return buffer.ToArray();
    }
}
