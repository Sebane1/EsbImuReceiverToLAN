using System.Numerics;

namespace EsbReceiverToLanAndroid.Views;

public class TrackerRotationView : GraphicsView
{
    public static readonly BindableProperty TrackerRotationProperty =
        BindableProperty.Create(nameof(TrackerRotation), typeof(Quaternion), typeof(TrackerRotationView), default(Quaternion), propertyChanged: OnRotationChanged);

    public Quaternion TrackerRotation
    {
        get => (Quaternion)GetValue(TrackerRotationProperty);
        set => SetValue(TrackerRotationProperty, value);
    }

    private static void OnRotationChanged(BindableObject bindable, object oldValue, object newValue)
    {
        if (bindable is TrackerRotationView view)
            view.Invalidate();
    }

    public TrackerRotationView()
    {
        HeightRequest = 40;
        WidthRequest = 40;
        Drawable = new TrackerRotationDrawable(this);
    }

    private class TrackerRotationDrawable : IDrawable
    {
        private readonly WeakReference<TrackerRotationView> _viewRef;

        // Cube vertices (centered at origin, ±1)
        private static readonly Vector3[] CubeVertices =
        {
            new(-1, -1, -1), new(1, -1, -1), new(1, 1, -1), new(-1, 1, -1), // back face
            new(-1, -1, 1), new(1, -1, 1), new(1, 1, 1), new(-1, 1, 1)     // front face
        };

        // 12 edges as vertex index pairs
        private static readonly (int A, int B)[] CubeEdges =
        {
            (0, 1), (1, 2), (2, 3), (3, 0),  // back
            (4, 5), (5, 6), (6, 7), (7, 4),  // front
            (0, 4), (1, 5), (2, 6), (3, 7)   // connecting
        };

        public TrackerRotationDrawable(TrackerRotationView view)
        {
            _viewRef = new WeakReference<TrackerRotationView>(view);
        }

        public void Draw(ICanvas canvas, RectF dirtyRect)
        {
            if (!_viewRef.TryGetTarget(out var view)) return;

            var q = view.TrackerRotation;
            if (q.LengthSquared() < 0.0001f) q = Quaternion.Identity;

            var rotated = new Vector3[8];
            for (int i = 0; i < 8; i++)
                rotated[i] = Vector3.Transform(CubeVertices[i], q);

            var cx = dirtyRect.Width / 2;
            var cy = dirtyRect.Height / 2;
            var scale = MathF.Min(dirtyRect.Width, dirtyRect.Height) / 4.5f;

            PointF Project(Vector3 v)
            {
                return new PointF(cx + v.X * scale, cy - v.Y * scale); // -Y so up is up on screen
            }

            var projected = new PointF[8];
            for (int i = 0; i < 8; i++)
                projected[i] = Project(rotated[i]);

            // Draw edges; front edges (higher Z) are brighter
            var edgesWithDepth = CubeEdges.Select(e =>
            {
                var a = rotated[e.A]; var b = rotated[e.B];
                var depth = (a.Z + b.Z) / 2;
                return (e.A, e.B, depth);
            }).OrderBy(x => x.depth).ToList();

            foreach (var (a, b, depth) in edgesWithDepth)
            {
                var alpha = 0.3f + 0.7f * (depth + 1) / 2; // depth -1..1 -> alpha 0.3..1
                canvas.StrokeColor = Colors.White.WithAlpha(alpha);
                canvas.StrokeSize = 1.5f;
                canvas.StrokeLineCap = LineCap.Round;
                canvas.DrawLine(projected[a], projected[b]);
            }
        }
    }
}
