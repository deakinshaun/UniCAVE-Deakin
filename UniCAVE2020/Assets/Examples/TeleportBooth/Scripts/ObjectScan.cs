using UnityEngine;
using System.Collections;
using Windows.Kinect;
using System.IO;
using System.Collections.Generic;

using Photon.Pun;
using Photon.Realtime;

public class ObjectScan : MonoBehaviour
{
    private KinectSensor sensor;
    private CoordinateMapper mapper;

    public class MeshDetails
    {
        public Mesh mesh;
        public Vector3[] vertices;
        public Vector2[] uv;
        public int[] triangles;
        public GameObject shape;
    };

    private MeshDetails mastermesh;

    // Size of Unity mesh is limited, so down sample to fit this limit.
    private const int _DownsampleSize = 2;
    private const double _DepthScale = 0.03f;
    
    public ColorSourceManager colorManager;
    public DepthSourceManager depthManager;

    public GameObject geometryPrototypeTemplate;

    public Vector3 scanPosition = new Vector3 (-10, 10, 0);

    private Dictionary<Player, MeshDetails> remoteMesh;

    private float timeSinceSend;
    private float sendInterval = 0.01f;

    // number of rows per rpc call
    private int rowSend = 2;
    private int currentrow = 0;

    void Start()
    {
        remoteMesh = new Dictionary<Player, MeshDetails>();
        timeSinceSend = sendInterval;
        sensor = KinectSensor.GetDefault();
        if (sensor != null)
        {
            mapper = sensor.CoordinateMapper;
            var frameDesc = sensor.DepthFrameSource.FrameDescription;

            mastermesh = new MeshDetails();
            mastermesh.mesh = new Mesh();
            CreateMesh(mastermesh, frameDesc.Width / _DownsampleSize, frameDesc.Height / _DownsampleSize, this.gameObject);

            if (sensor.IsOpen)
            {
                sensor.Open();
            }
        }
    }

    void CreateMesh(MeshDetails mesh, int width, int height, GameObject targetMesh)
    {
        targetMesh.GetComponent<MeshFilter>().mesh = mesh.mesh;

        mesh.vertices = new Vector3[width * height];
        mesh.uv = new Vector2[width * height];
        mesh.triangles = new int[6 * ((width - 1) * (height - 1))];

        int triangleIndex = 0;
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int index = (y * width) + x;

                mesh.vertices[index] = new Vector3(x, -y, 0);
                mesh.uv[index] = new Vector2(((float)x / (float)width), ((float)y / (float)height));

                // Skip the last row/col
                if (x != (width - 1) && y != (height - 1))
                {
                    int topLeft = index;
                    int topRight = topLeft + 1;
                    int bottomLeft = topLeft + width;
                    int bottomRight = bottomLeft + 1;

                    mesh.triangles[triangleIndex++] = topLeft;
                    mesh.triangles[triangleIndex++] = topRight;
                    mesh.triangles[triangleIndex++] = bottomLeft;
                    mesh.triangles[triangleIndex++] = bottomLeft;
                    mesh.triangles[triangleIndex++] = topRight;
                    mesh.triangles[triangleIndex++] = bottomRight;
                }
            }
        }

        mesh.mesh.vertices = mesh.vertices;
        mesh.mesh.uv = mesh.uv;
        mesh.mesh.triangles = mesh.triangles;
        mesh.mesh.RecalculateNormals();
    }

    [PunRPC]
    void UpdateGeometry(int w, int h, int row, int numrows, int[] depths, Vector3 position, PhotonMessageInfo info)
    {
        Debug.Log(string.Format("UpdateGeometry {0} {1}x{2} {3}", depths.Length, w, h, info.Sender));

        if (!remoteMesh.ContainsKey (info.Sender))
        {
            GameObject shape = Instantiate(geometryPrototypeTemplate);
            MeshDetails md = new MeshDetails();
            md.mesh = shape.GetComponent<MeshFilter>().mesh;
            md.shape = shape;
            remoteMesh[info.Sender] = md;
            CreateMesh(md, w / _DownsampleSize, h / _DownsampleSize, shape);
            shape.transform.position = position;
        }
        RefreshData(remoteMesh[info.Sender], w, h, row, numrows, depths, 0, 0);
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
        var frameDesc = sensor.DepthFrameSource.FrameDescription;
        RefreshData(mastermesh, frameDesc.Width, frameDesc.Height, 0, frameDesc.Height, data, colorManager.ColorWidth, colorManager.ColorHeight);

        // Send to any receivers.
        timeSinceSend -= Time.deltaTime;
        if (timeSinceSend < 0.0f)
        {
            timeSinceSend = sendInterval;
            PhotonView photonView = PhotonView.Get(this);
             
            int w = frameDesc.Width;
            int h = frameDesc.Height;
            int[] rowdata = new int[w * rowSend];
            for (int j = 0; j < rowSend * w; j++)
            {
                int p = currentrow * w + j;
                if (p < data.Length)
                {
                    rowdata[j] = data[p];
                }
            }
            photonView.RPC("UpdateGeometry", RpcTarget.All, w, h, currentrow, rowSend, rowdata, scanPosition);
            currentrow += rowSend;
            if (currentrow > h)
            {
                currentrow = 0;
            }
        }

        if (Input.GetAxis ("Fire1") > 0.0f)
        {
            exportObj("scan");
        }
    }
    
    private void RefreshData(MeshDetails mesh, int width, int height, int row, int numrows, int[] depthData, int colorWidth, int colorHeight)
    {
        ColorSpacePoint[] colorSpace = new ColorSpacePoint[depthData.Length];
        ushort [] shortData = new ushort[depthData.Length];
        for (int i = 0; i < depthData.Length; i++)
        {
            shortData[i] = (ushort)depthData[i];
        }
        if (colorWidth > 0)
        {
            mapper.MapDepthFrameToColorSpace(shortData, colorSpace);
        }
        
        for (int y = 0; y < height - (_DownsampleSize - 1); y += _DownsampleSize)
        {
            for (int x = 0; x < width - (_DownsampleSize - 1); x += _DownsampleSize)
            {
                if ((y >= row) && (y < row + numrows))
                {
                    int rowy = y - row;
                    int indexX = x / _DownsampleSize;
                    int indexY = y / _DownsampleSize;
                    int swidth = width / _DownsampleSize;
                    int smallIndex = indexY * swidth + indexX;

                    double avg = GetAvg(depthData, x, rowy, width, height);

                    avg = avg * _DepthScale;

                    mesh.vertices[smallIndex].z = (float)avg;

                    // Update UV mapping with CDRP
                    if (colorWidth > 0)
                    {
                        var colorSpacePoint = colorSpace[(y * width) + x];
                        mesh.uv[smallIndex] = new Vector2(colorSpacePoint.X / colorWidth, colorSpacePoint.Y / colorHeight);
                    }
                }
            }
        }

        mesh.mesh.vertices = mesh.vertices;
        mesh.mesh.uv = mesh.uv;
        mesh.mesh.triangles = mesh.triangles;
        mesh.mesh.RecalculateNormals();
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
            foreach (Vector3 v in mastermesh.vertices)
            {
                file.WriteLine("v " + v.x + " " + v.y + " " + v.z);
            }
            foreach (Vector2 v in mastermesh.uv)
            {
                file.WriteLine("vt " + v.x + " " + v.y);
            }
            foreach (Vector3 v in mastermesh.mesh.normals)
            {
                file.WriteLine("vn " + v.x + " " + v.y + " " + v.z);
            }
            file.WriteLine("usemtl meshMaterial");
            file.WriteLine("s off");
            for (int i = 0; i < mastermesh.triangles.Length; i += 3)
            {
                file.WriteLine("f " + (1 + mastermesh.triangles[i + 0]) + "/" + (1 + mastermesh.triangles[i + 0]) + "/" + (1 + mastermesh.triangles[i + 0]) + " " +
                                      (1 + mastermesh.triangles[i + 1]) + "/" + (1 + mastermesh.triangles[i + 1]) + "/" + (1 + mastermesh.triangles[i + 1]) + " " +
                                      (1 + mastermesh.triangles[i + 2]) + "/" + (1 + mastermesh.triangles[i + 2]) + "/" + (1 + mastermesh.triangles[i + 2]));
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
