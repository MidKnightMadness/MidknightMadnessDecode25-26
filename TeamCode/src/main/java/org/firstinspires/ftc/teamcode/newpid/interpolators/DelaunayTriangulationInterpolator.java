package org.firstinspires.ftc.teamcode.newpid;

import android.os.Environment;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class DelaunayTriangulation implements Interpolator2D<DelaunayTriangulation> {
    private final List<Triangle> triangles = new ArrayList<>();
    private final List<Point3D> points = new ArrayList<>();
    private final static double EPSILON = 1e-9;

    // ----------------------------
    // Data classes
    // ----------------------------
    public static class Triangle {
        public Point3D a, b, c;

        public Triangle() {}
        public Triangle(Point3D a, Point3D b, Point3D c) { this.a = a; this.b = b; this.c = c; }

        public Triangle(Point3D p1, Point3D p2, Point3D p3, double epsilon) {
            if ((p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x) < 0) {
                this.a = p1; this.b = p3; this.c = p2;
            } else {
                this.a = p1; this.b = p2; this.c = p3;
            }
        }

        public boolean circumcircleContains(Point3D p, double epsilon) {
            double ax_p = a.x - p.x, ay_p = a.y - p.y;
            double bx_p = b.x - p.x, by_p = b.y - p.y;
            double cx_p = c.x - p.x, cy_p = c.y - p.y;

            double det = (ax_p*ax_p + ay_p*ay_p)*(bx_p*cy_p - by_p*cx_p)
                    - (bx_p*bx_p + by_p*by_p)*(ax_p*cy_p - ay_p*cx_p)
                    + (cx_p*cx_p + cy_p*cy_p)*(ax_p*by_p - ay_p*bx_p);

            return det > epsilon;
        }

        public boolean containsPoint(double px, double py) {
            double det = (b.y - c.y)*(a.x - c.x) + (c.x - b.x)*(a.y - c.y);
            double w1 = ((b.y - c.y)*(px - c.x) + (c.x - b.x)*(py - c.y)) / det;
            double w2 = ((c.y - a.y)*(px - c.x) + (a.x - c.x)*(py - c.y)) / det;
            double w3 = 1.0 - w1 - w2;
            return w1 >= -EPSILON && w2 >= -EPSILON && w3 >= -EPSILON;
        }

        public double interpolateZ(double px, double py) {
            double det = (b.y - c.y)*(a.x - c.x) + (c.x - b.x)*(a.y - c.y);
            double w1 = ((b.y - c.y)*(px - c.x) + (c.x - b.x)*(py - c.y)) / det;
            double w2 = ((c.y - a.y)*(px - c.x) + (a.x - c.x)*(py - c.y)) / det;
            return w1*a.z + w2*b.z + (1-w1-w2)*c.z;
        }
    }

    private static class Edge {
        Point3D v1, v2;
        Edge(Point3D v1, Point3D v2) { this.v1 = v1; this.v2 = v2; }
        boolean isSame(Edge e) { return (v1 == e.v1 && v2 == e.v2) || (v1 == e.v2 && v2 == e.v1); }
    }

    // ----------------------------
    // Constructors
    // ----------------------------
    public DelaunayTriangulation() {}

    public DelaunayTriangulation(List<Point3D> inputPoints) {
        this.points.addAll(inputPoints);
        if (points.size() < 3) return;

        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (Point3D p : points) {
            minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
            minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
        }
        double dx = maxX - minX, dy = maxY - minY, dmax = Math.max(dx, dy);
        Point3D stA = new Point3D(minX - 20*dmax, minY - dmax, 0);
        Point3D stB = new Point3D(maxX + 20*dmax, minY - dmax, 0);
        Point3D stC = new Point3D(minX + dx/2, maxY + 20*dmax, 0);
        triangles.add(new Triangle(stA, stB, stC, EPSILON));

        for (Point3D p : points) {
            List<Triangle> badTriangles = new ArrayList<>();
            for (Triangle t : triangles) if (t.circumcircleContains(p, EPSILON)) badTriangles.add(t);

            List<Edge> polygon = new ArrayList<>();
            for (Triangle t : badTriangles) {
                addEdgeUnique(polygon, new Edge(t.a, t.b));
                addEdgeUnique(polygon, new Edge(t.b, t.c));
                addEdgeUnique(polygon, new Edge(t.c, t.a));
            }
            triangles.removeAll(badTriangles);
            for (Edge e : polygon) triangles.add(new Triangle(e.v1, e.v2, p, EPSILON));
        }

        triangles.removeIf(t -> t.a == stA || t.a == stB || t.a == stC
                || t.b == stA || t.b == stB || t.b == stC
                || t.c == stA || t.c == stB || t.c == stC);
    }

    @Override
    public void toFile(String fileName) throws IOException {
        try {
            JSONObject obj = new JSONObject();

            // Save points
            JSONArray pts = new JSONArray();
            for (Point3D p : points) {
                JSONObject pointObj = new JSONObject();
                pointObj.put("x", p.x);
                pointObj.put("y", p.y);
                pointObj.put("z", p.z);
                pts.put(pointObj);
            }
            obj.put("points", pts);

            // Save triangles
            JSONArray tris = new JSONArray();
            for (Triangle t : triangles) {
                JSONObject triObj = new JSONObject();

                JSONObject a = new JSONObject();
                a.put("x", t.a.x);
                a.put("y", t.a.y);
                a.put("z", t.a.z);

                JSONObject b = new JSONObject();
                b.put("x", t.b.x);
                b.put("y", t.b.y);
                b.put("z", t.b.z);

                JSONObject c = new JSONObject();
                c.put("x", t.c.x);
                c.put("y", t.c.y);
                c.put("z", t.c.z);

                triObj.put("a", a);
                triObj.put("b", b);
                triObj.put("c", c);

                tris.put(triObj);
            }
            obj.put("triangles", tris);

            // Write JSON to file
            File file = new File(Environment.getExternalStorageDirectory(), fileName);
            ReadWriteFile.writeFile(file, obj.toString(4)); // pretty-print with 4-space indent

        } catch (JSONException e) {
            throw new IOException("Failed to serialize triangulation to JSON", e);
        }
    }

    // ----------------------------
    // Factory method: load from file
    // ----------------------------
    @Override
    public DelaunayTriangulation fromFile(String fileName) throws IOException, JSONException {
        DelaunayTriangulation instance = new DelaunayTriangulation();

        File file = new File(Environment.getExternalStorageDirectory(), fileName);
        if (!file.exists()) throw new FileNotFoundException(file.getAbsolutePath() + " not found");

        String json = ReadWriteFile.readFile(file);
        if (json == null) throw new IOException("Failed to read file: " + file.getAbsolutePath());

        JSONObject obj = new JSONObject(json);

        JSONArray pts = obj.getJSONArray("points");
        for (int i=0; i<pts.length(); i++) {
            JSONObject p = pts.getJSONObject(i);
            instance.points.add(new Point3D(p.getDouble("x"), p.getDouble("y"), p.getDouble("z")));
        }

        JSONArray tris = obj.getJSONArray("triangles");
        for (int i=0; i<tris.length(); i++) {
            JSONObject t = tris.getJSONObject(i);
            JSONObject a = t.getJSONObject("a");
            JSONObject b = t.getJSONObject("b");
            JSONObject c = t.getJSONObject("c");
            instance.triangles.add(new Triangle(
                    new Point3D(a.getDouble("x"), a.getDouble("y"), a.getDouble("z")),
                    new Point3D(b.getDouble("x"), b.getDouble("y"), b.getDouble("z")),
                    new Point3D(c.getDouble("x"), c.getDouble("y"), c.getDouble("z"))
            ));
        }
        return instance;
    }

    // ----------------------------
    // Public methods
    // ----------------------------
    public double getZ(double x, double y) {
        for (Triangle t : triangles) {
            if (t.containsPoint(x, y)) return t.interpolateZ(x, y);
        }

        if (points.isEmpty()) return 0;
        Point3D nearest = points.get(0);
        double minDistSq = Double.MAX_VALUE;
        for (Point3D p : points) {
            double d2 = Math.pow(x - p.x, 2) + Math.pow(y - p.y, 2);
            if (d2 < minDistSq) { minDistSq = d2; nearest = p; }
        }
        return nearest.z;
    }

    public List<Triangle> getTriangles() { return triangles; }
    public List<Point3D> getPoints() { return points; }

    // ----------------------------
    // Internal helpers
    // ----------------------------
    private void addEdgeUnique(List<Edge> poly, Edge e) {
        for (int i = 0; i < poly.size(); i++) {
            if (poly.get(i).isSame(e)) { poly.remove(i); return; }
        }
        poly.add(e);
    }

    public void telemetryPoints(Telemetry telemetry) {
        telemetry.addData("Points", points);
    }
}
