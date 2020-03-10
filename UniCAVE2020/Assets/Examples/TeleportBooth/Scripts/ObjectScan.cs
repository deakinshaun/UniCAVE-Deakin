using UnityEngine;
using System.Collections;
using Windows.Kinect;
using System.IO;

using Photon.Pun;
using Photon.Realtime;

public class ObjectScan : MonoBehaviour
{
    private KinectSensor sensor;
    private CoordinateMapper mapper;

    private Mesh mesh;
    private Vector3[] vertices;
    private Vector2[] uv;
    private int[] triangles;
    
    // Size of Unity mesh is limited, so down sample to fit this limit.
    private const int _DownsampleSize = 2;
    private const double _DepthScale = 0.03f;
    
    public ColorSourceManager colorManager;
    public DepthSourceManager depthManager;

    public GameObject geometryPrototypeTemplate;

    private GameObject shape = null;

    void Start()
    {
        sensor = KinectSensor.GetDefault();
        if (sensor != null)
        {
            mapper = sensor.CoordinateMapper;
            var frameDesc = sensor.DepthFrameSource.FrameDescription;

            mesh = new Mesh();
            CreateMesh(mesh, frameDesc.Width / _DownsampleSize, frameDesc.Height / _DownsampleSize, this.gameObject);

            if (sensor.IsOpen)
            {
                sensor.Open();
            }
        }
    }

    void CreateMesh(Mesh mesh, int width, int height, GameObject targetMesh)
    {
        targetMesh.GetComponent<MeshFilter>().mesh = mesh;

        vertices = new Vector3[width * height];
        uv = new Vector2[width * height];
        triangles = new int[6 * ((width - 1) * (height - 1))];

        int triangleIndex = 0;
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int index = (y * width) + x;

                vertices[index] = new Vector3(x, -y, 0);
                uv[index] = new Vector2(((float)x / (float)width), ((float)y / (float)height));

                // Skip the last row/col
                if (x != (width - 1) && y != (height - 1))
                {
                    int topLeft = index;
                    int topRight = topLeft + 1;
                    int bottomLeft = topLeft + width;
                    int bottomRight = bottomLeft + 1;

                    triangles[triangleIndex++] = topLeft;
                    triangles[triangleIndex++] = topRight;
                    triangles[triangleIndex++] = bottomLeft;
                    triangles[triangleIndex++] = bottomLeft;
                    triangles[triangleIndex++] = topRight;
                    triangles[triangleIndex++] = bottomRight;
                }
            }
        }

        mesh.vertices = vertices;
        mesh.uv = uv;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
    }

    [PunRPC]
    void UpdateGeometry(int w, int h, int[] depths, PhotonMessageInfo info)
    {
        Debug.Log(string.Format("UpdateGeometry {0} {1}x{2} {3}", depths.Length, w, h, info.Sender));

        if (shape == null)
        {
            shape = Instantiate(geometryPrototypeTemplate);
            CreateMesh(shape.GetComponent<MeshFilter>().mesh, w, h, shape);
        }
        RefreshData(shape.GetComponent<MeshFilter>().mesh, depths, 0, 0);
    }

    void Update()
    {
        if (sensor == null)
        {
            return;
        }
        gameObject.GetComponent<MeshRenderer> ().material.mainTexture = colorManager.GetColorTexture();
        ushort [] depthData = depthManager.GetData();
        int[] data = new int[depthData.Length];
        for (int i = 0; i < depthData.Length; i++)
        {
            data[i] = depthData[i];
        }
        RefreshData(mesh, data, colorManager.ColorWidth, colorManager.ColorHeight);

        // Send to any receivers.
        PhotonView photonView = PhotonView.Get(this);
        var frameDesc = sensor.DepthFrameSource.FrameDescription;
        photonView.RPC("UpdateGeometry", RpcTarget.All, frameDesc.Width / _DownsampleSize, frameDesc.Height / _DownsampleSize, data);


        if (Input.GetAxis ("Fire1") > 0.0f)
        {
            exportObj("scan");
        }
    }
    
    private void RefreshData(Mesh mesh, int[] depthData, int colorWidth, int colorHeight)
    {
        var frameDesc = sensor.DepthFrameSource.FrameDescription;
        
        ColorSpacePoint[] colorSpace = new ColorSpacePoint[depthData.Length];
        ushort [] shortData = new ushort[depthData.Length];
        for (int i = 0; i < depthData.Length; i++)
        {
            shortData[i] = (ushort)depthData[i];
        }
        mapper.MapDepthFrameToColorSpace(shortData, colorSpace);
        
        for (int y = 0; y < frameDesc.Height - (_DownsampleSize - 1); y += _DownsampleSize)
        {
            for (int x = 0; x < frameDesc.Width - (_DownsampleSize - 1); x += _DownsampleSize)
            {
                int indexX = x / _DownsampleSize;
                int indexY = y / _DownsampleSize;
                int width = frameDesc.Width / _DownsampleSize;
                int smallIndex = indexY * width + indexX;
                
                double avg = GetAvg(depthData, x, y, frameDesc.Width, frameDesc.Height);
                
                avg = avg * _DepthScale;
                
                vertices[smallIndex].z = (float)avg;
                
                // Update UV mapping with CDRP
                var colorSpacePoint = colorSpace[(y * frameDesc.Width) + x];
                if (colorWidth > 0)
                {
                    uv[smallIndex] = new Vector2(colorSpacePoint.X / colorWidth, colorSpacePoint.Y / colorHeight);
                }
            }
        }
        
        mesh.vertices = vertices;
        mesh.uv = uv;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
    }
    
    private double GetAvg(int[] depthData, int x, int y, int width, int height)
    {
        double sum = 0.0;
        int count = 0;
        for (int y1 = y; y1 < y + _DownsampleSize; y1++)
        {
            for (int x1 = x; x1 < x + _DownsampleSize; x1++)
            {
                if ((x < width) && (y < height))
                {
                    int fullIndex = (y1 * width) + x1;

                    if (depthData[fullIndex] == 0)
                    {
                        sum += 4500;
                    }
                    else
                    {
                        sum += depthData[fullIndex];
                    }
                    count++;
                }
            }
        }

        return sum / count;
    }

    void exportObj(string filenameBase)
    {
        Debug.Log("Saving scan");

        // Write obj file
        using (System.IO.StreamWriter file = new System.IO.StreamWriter(filenameBase + ".obj"))
        {
            file.WriteLine("mtllib " + filenameBase + ".mtl");
            file.WriteLine("o mesh");
            foreach (Vector3 v in vertices)
            {
                file.WriteLine("v " + v.x + " " + v.y + " " + v.z);
            }
            foreach (Vector2 v in uv)
            {
                file.WriteLine("vt " + v.x + " " + v.y);
            }
            foreach (Vector3 v in mesh.normals)
            {
                file.WriteLine("vn " + v.x + " " + v.y + " " + v.z);
            }
            file.WriteLine("usemtl meshMaterial");
            file.WriteLine("s off");
            for (int i = 0; i < triangles.Length; i += 3)
            {
                file.WriteLine("f " + (1 + triangles[i + 0]) + "/" + (1 + triangles[i + 0]) + "/" + (1 + triangles[i + 0]) + " " +
                                      (1 + triangles[i + 1]) + "/" + (1 + triangles[i + 1]) + "/" + (1 + triangles[i + 1]) + " " +
                                      (1 + triangles[i + 2]) + "/" + (1 + triangles[i + 2]) + "/" + (1 + triangles[i + 2]));
            }
        }

        // Write mtl file
        using (System.IO.StreamWriter file = new System.IO.StreamWriter(filenameBase + ".mtl"))
        {
            file.WriteLine("newmtl meshMaterial");
            file.WriteLine("o mesh");
            file.WriteLine("Ns 96.078431");
            file.WriteLine("Ka 1.000000 1.000000 1.000000");
            file.WriteLine("Kd 0.640000 0.640000 0.640000");
            file.WriteLine("Ks 0.500000 0.500000 0.500000");
            file.WriteLine("Ke 0.000000 0.000000 0.000000");
            file.WriteLine("Ni 1.000000");
            file.WriteLine("d 1.000000");
            file.WriteLine("illum 2");
            file.WriteLine("map_Kd " + filenameBase + ".jpg");
        }
        File.WriteAllBytes (filenameBase + ".jpg", ImageConversion.EncodeToJPG (colorManager.GetColorTexture()));
    }
}
