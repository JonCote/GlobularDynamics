using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;
using UnityEngine;
using TMPro;
using static ParticleSim;

public class ParticleSim : MonoBehaviour
{
    public struct LookupGrid
    {
        public uint key;
        public uint index;

        public LookupGrid(uint key, uint index)
        {
            this.key = key;
            this.index = index;
        }
    }

    public struct LookupGridCompare : IComparer<LookupGrid>
    {
        public int Compare(LookupGrid x, LookupGrid y)
        {
            return x.key.CompareTo(y.key);
        }
    }

    [SerializeField] private Mesh _mesh;
    [SerializeField] private Material _material;
    public TMP_Text canvasText;


    private List<Transform> _objects;
    private NativeArray<float3> _objectPos;
    private NativeArray<float> _objectR;
    
    private NativeArray<Matrix4x4> _matrices;
    private NativeArray<float3> _positions;
    private NativeArray<float3> _velocities;
    private NativeArray<float3> _forces;
    private NativeArray<LookupGrid> _lookupGrid;
    private NativeArray<int> _startIndex;


    (int, int)[] cellOffsets = new (int, int)[]
    {
        (0, 0), (1, 0), (0, 1), (1, 1),
        (0, -1), (-1, 0), (-1, -1), (-1, 1), (1, -1)
    };

    private RenderParams _rp;

    private float r0 = 0.3f;
    private float r;
    public static float3 sphereScale { get; } = new float3(0.3f, 0.3f, 0.3f);
    //public float3 sphereScale;
    [field: SerializeField] public Color[] ColorArray { get; private set; }
    public int numParticles = 1;
    public float ParticleMass = 1.0f;
    private int particleCount = 0;
    private float nextSpawn = 0.0f;
    private float dt = 0.0f;

    public float gravity = 9.8f;
    //public float collisionDamping = 0.5f;
    public float dragC = 0.1f;
    public float b1;
    public float bd;
    public float cr;
    public float cd;

    float rmin = 0.0f;
    float rmax = 0.0f;
    float denom_sr;
    float denom_sd;
    int m = 5;
    int n = 3;
    float approxR0 = 0.0f;

    public bool particleStream = false;
    public List<GameObject> emitters = new List<GameObject>();
    public float spawnRate = 1.0f;
    public float3 InitialVelocity = new float3(0, 0, 0);
    public bool randomEmit = false;
    public float particleSpacing = 1.0f;

    public float3 boundsSize = new float3(10, 10, 0);



    // Start is called before the first frame update
    void Start()
    {

        _positions = new NativeArray<float3>(numParticles, Allocator.Persistent);
        _matrices = new NativeArray<Matrix4x4>(numParticles, Allocator.Persistent);
        _forces = new NativeArray<float3>(numParticles, Allocator.Persistent);
        _velocities = new NativeArray<float3>(numParticles, Allocator.Persistent);

        _lookupGrid = new NativeArray<LookupGrid>(numParticles, Allocator.Persistent);
        _startIndex = new NativeArray<int>(numParticles, Allocator.Persistent);

        _objects = GetObjects(transform, false);

        int objectCount = _objects.Count;
        _objectPos = new NativeArray<float3>(objectCount, Allocator.Persistent);
        _objectR = new NativeArray<float>(objectCount, Allocator.Persistent);

        if (canvasText != null)
        {
            canvasText.text = "Particle Count: " + particleCount;
        }

        for (int i = 0; i < objectCount; i++)
        {
            _objectPos[i] = new float3(_objects[i].position.x, _objects[i].position.y, _objects[i].position.z);
            _objectR[i] = _objects[i].localScale.x / 2;
        }


        nextSpawn = spawnRate;
        r = r0 / 2;

        for (int i = 0; i < numParticles; i++)
        {
            _lookupGrid[i] = new LookupGrid(uint.MaxValue, uint.MaxValue);
            _startIndex[i] = int.MaxValue;
            
            _positions[i] = new float3(-100.0f, -100.0f, -100.0f);
            _velocities[i] = new float3(0.0f, 0.0f, 0.0f);
            _forces[i] = new float3(0.0f, 0.0f, 0.0f);
        }

        _rp = new RenderParams(_material);


        float deltaTime = 0.01f;
        Time.fixedDeltaTime = deltaTime;


        denom_sr = ((cr) * (cr)) * ((r + r) * (r + r));
        denom_sd = ((cd) * (cd)) * ((r + r) * (r + r));


        rmin = r;
        rmax = cr * r0;
        if (cd * r0 > rmax)
        {
            rmax = cd * r0;
        }

        approxR0 = r0 * 0.05f;

        if (!particleStream)
        {
            gridFormation();
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        dt += Time.fixedDeltaTime;
        if (canvasText != null)
        {
            canvasText.text = "Particle Count: " + particleCount;
        }

        if (dt > nextSpawn && particleStream)
        {
            nextSpawn += spawnRate;
            streamParticles();
        }

       // updateGrid();

        System.Threading.Tasks.Parallel.For(0, particleCount, i =>
        {
            _forces[i] = new float3(0.0f, -1.0f, 0.0f) * gravity * ParticleMass;
            gd_forces(i);
        });

        System.Threading.Tasks.Parallel.For(0, particleCount, i =>
        {
            ResolveBoundCollision(i);
        });

        for (int i = 0; i < particleCount; i++)
        {
            ResolveObjectCollision(i);
        }

        integrateSymplectic(Time.fixedDeltaTime);
    }

    void Update()
    {
        Graphics.RenderMeshInstanced(_rp, _mesh, 0, _matrices);
    }

    private void OnDestroy()
    {
        _positions.Dispose();
        _matrices.Dispose();
        _forces.Dispose();
        _velocities.Dispose();
        _lookupGrid.Dispose();
        _startIndex.Dispose();
        _objectPos.Dispose();
        _objectR.Dispose();
    }


    void integrateSymplectic(float frameTime)
    {
        System.Threading.Tasks.Parallel.For(0, particleCount, i =>
        {
            _velocities[i] += (_forces[i] * frameTime) / ParticleMass;
            _positions[i] += _velocities[i] * frameTime;
            _velocities[i] *= 1.0f - dragC;

            _matrices[i] = Matrix4x4.TRS(_positions[i], Quaternion.identity, sphereScale);
        });
    }


    void gd_forces(int p)
    {
        float3 P = new float3(0, 0, 0);
        float3 V = new float3(0, 0, 0);
        float D = 0.0f;
        float sr = 0.0f;
        float sd = 0.0f;

        float b2 = b1 / ((r0) * (r0));
        for (int i = 0; i < particleCount; i++)
        {
            float3 f = new float3(0, 0, 0);

            if (i == p)
            {
                continue;
            }
            else
            {

                P = _positions[p] - _positions[i];
                V = _velocities[p] - _velocities[i];
                D = math.length(P);

                if (D < rmin)
                {
                    D = rmin;
                }

                if (D >= (r0 - approxR0) && D <= (r0 + approxR0))
                {
                    continue;
                }

                float numer = D * D;
                //Debug.Log("D: " + D + " D^2: " + numer + " denom_sr: " + denom_sr + " denom_sd: " + denom_sd + " cr: " + cr + " r: " + r);

                if (numer < denom_sr)
                {
                    sr = 1 - (numer / denom_sr);
                }
                else
                {
                    sr = 0;
                }

                if (numer < denom_sd)
                {

                    sd = 1 - (numer / denom_sd);
                }
                else
                {
                    //Debug.Log("numer: " + numer + " denom_sd: " + denom_sd);
                    continue;
                }

                float repulsive_force;
                float attraction_force;

                if (sr == 0)
                {
                    f = P * (sd * bd * ((math.dot(V, P)) / ((D) * (D))));
                    _forces[p] -= f;
                }
                else
                {
                    repulsive_force = b1 / Mathf.Pow((D), m);
                    attraction_force = b2 / Mathf.Pow((D), n);

                    f = P * ((sr * (repulsive_force - attraction_force)) - (sd * bd * ((math.dot(V, P)) / ((D) * (D)))));

                    _forces[p] += f;
                }
            }
        }
    }

    void streamParticles()
    {
        if (randomEmit)
        {
            randomStream();
        }
        else
        {
            foreach (GameObject emitter in emitters)
            {
                if (particleCount < numParticles)
                {
                    _positions[particleCount] = new float3(emitter.transform.position.x, emitter.transform.position.y, emitter.transform.position.z);
                    _velocities[particleCount] = InitialVelocity;
                    _forces[particleCount] = new float3(0, 0, 0);
                    particleCount++;
                }
            }
        }
    }

    void randomStream()
    {
        if (particleCount < numParticles)
        {
            GameObject randEmitter = emitters[UnityEngine.Random.Range(0, emitters.Count)];
            _positions[particleCount] = new float3(randEmitter.transform.position.x, randEmitter.transform.position.y, randEmitter.transform.position.z);
            _velocities[particleCount] = InitialVelocity;
            _forces[particleCount] = new float3(0, 0, 0);
            particleCount++;
        }
    }



    void gridFormation()
    {
        int particlesPerRow = (int)Mathf.Sqrt(numParticles);
        int particlesPerColumn = (numParticles - 1) / particlesPerRow + 1;
        float spacing = particleSpacing;

        particleCount = numParticles;

        for (int i = 0; i < particleCount; i++)
        {
            float x = (i % particlesPerRow - particlesPerRow / 2f) * spacing;
            float y = (i / particlesPerRow - particlesPerColumn / 2f) * spacing;

            _positions[i] = new float3(x, y, 0);
            _velocities[i] = InitialVelocity;
            _forces[i] = float3.zero;
        }
    }


    void boundCollision(float3 boundPoint, int p)
    {
        float3 P = new float3(0, 0, 0);
        float3 V = new float3(0, 0, 0);
        float3 f = new float3(0, 0, 0);
        float D = 0.0f;
        float sr = 0.0f;
        float sd = 0.0f;
        float objb2 = 0.0f;
        float objb1 = 10.0f;
        float objCr = 1.0f;
        float objCd = 1.0f;
        float objbd = 5.0f * cd;

        float objR = 0.5f;

        objb2 = objb1 / (Mathf.Pow(objR + r, 2));


        P = _positions[p] - boundPoint;
        V = _velocities[p];
        D = math.length(P);

        if (D > (objR + r))
        {
            return;
        }

        // need to calculate sr, sd
        float d_sr = ((objCr) * (objCr)) * (Mathf.Pow(objR + r, 2));
        float d_sd = ((objCd) * (objCd)) * (Mathf.Pow(objR + r, 2));
        float numer = D * D;
        //Debug.Log("D: " + D + " D^2: " + numer + " denom_sr: " + denom_sr + " denom_sd: " + denom_sd + " cr: " + cr + " r: " + r);

        if (numer < d_sr)
        {
            sr = 1 - (numer / d_sr);
        }
        else
        {
            sr = 0;
        }

        if (numer < d_sd)
        {

            sd = 1 - (numer / d_sd);
        }
        else
        {
            //Debug.Log("numer: " + numer + " denom_sd: " + denom_sd);
            return;
        }

        float repulsive_force;
        float attraction_force;

        if (sr == 0)
        {
            f = P * (sd * objbd * ((math.dot(V, P)) / ((D) * (D))));
            _forces[p] -= f;
        }
        else
        {
            repulsive_force = objb1 / Mathf.Pow((D), m);
            attraction_force = objb2 / Mathf.Pow((D), n);

            f = P * ((sr * (repulsive_force - attraction_force)) - (sd * objbd * ((math.dot(V, P)) / ((D) * (D)))));

            _forces[p] += f;

        }
    }

    void ResolveBoundCollision(int i)
    {
        float3 halfBounds = (boundsSize / 2) - (new float3(r, r, r));

     
        if (System.Math.Abs(_positions[i].x) > halfBounds.x)
        {
            //_positions[i] = new float3((halfBounds.x * System.Math.Sign(_positions[i].x)), _positions[i].y, _positions[i].z);
            //_velocities[i] *= new float3(-1.0f, 1.0f, 1.0f) * collisionDamping;

            float3[] walls = new float3[] { new float3(boundsSize.x / 2, 0, 0), new float3(-boundsSize.x / 2, 0, 0) };
            float3 halfExtents = new float3(0, boundsSize.y / 2, boundsSize.z / 2);
            float3[] rOffsets = new float3[] { new float3(0.45f, 0, 0), new float3(-0.45f, 0, 0) };


            for (int w = 0; w < 2; w++)
            {

                float3 wallPos = walls[w];

                float3 D = _positions[i] - wallPos;

                float3 left_halfExtent = -halfExtents;
                float3 right_halfExtent = halfExtents;


                float3 clamped = math.clamp(D, left_halfExtent, right_halfExtent);

                float3 closestPoint = (wallPos + clamped);

                float dst = math.length(closestPoint - _positions[i]);

                if (dst < r)
                {
                    boundCollision((closestPoint + rOffsets[w]), i);
                    break;
                }
            }
        }
        
        if (System.Math.Abs(_positions[i].y) > halfBounds.y)
        {

            float3[] walls = new float3[] { new float3(0, boundsSize.y / 2, 0), new float3(0, -boundsSize.y / 2, 0) };
            float3 halfExtents = new float3(boundsSize.x / 2, 0, boundsSize.z / 2);
            float3[] rOffsets = new float3[] { new float3(0, 0.45f, 0), new float3(0, -0.45f, 0) };


            for (int w = 0; w < 2; w++)
            {

                float3 wallPos = walls[w];

                float3 D = _positions[i] - wallPos;

                float3 left_halfExtent = -halfExtents;
                float3 right_halfExtent = halfExtents;


                float3 clamped = math.clamp(D, left_halfExtent, right_halfExtent);

                float3 closestPoint = (wallPos + clamped);

                float dst = math.length(closestPoint - _positions[i]);

                if (dst < r)
                {
                    boundCollision((closestPoint + rOffsets[w]), i);
                    break;
                }
            }
        }

        if (System.Math.Abs(_positions[i].z) > halfBounds.z)
        {
            float3[] walls = new float3[] { new float3(0, 0, boundsSize.z / 2), new float3(0, 0, -boundsSize.z / 2) };
            float3 halfExtents = new float3(boundsSize.x / 2, boundsSize.y / 2, 0);
            float3[] rOffsets = new float3[] { new float3(0, 0, 0.45f), new float3(0, 0, -0.45f) };


            for (int w = 0; w < 2; w++)
            {

                float3 wallPos = walls[w];

                float3 D = _positions[i] - wallPos;

                float3 left_halfExtent = -halfExtents;
                float3 right_halfExtent = halfExtents;


                float3 clamped = math.clamp(D, left_halfExtent, right_halfExtent);

                float3 closestPoint = (wallPos + clamped);

                float dst = math.length(closestPoint - _positions[i]);

                if (dst < r)
                {
                    boundCollision((closestPoint + rOffsets[w]), i);
                    break;
                }
            }
        }
    }


    void ResolveObjectCollision(int p)
    {
        for (int i = 0; i < _objectPos.Length; i++)
        {

            float3 P = new float3(0, 0, 0);
            float3 V = new float3(0, 0, 0);
            float3 f = new float3(0, 0, 0);
            float D = 0.0f;
            float sr = 0.0f;
            float sd = 0.0f;
            float objb2 = 0.0f;
            float objb1 = 10.0f;
            float objCr = 1.0f;
            float objCd = 1.0f;
            float objbd = 5.0f * cd;

            float objR = _objectR[i];

            objb2 = objb1 / (Mathf.Pow(objR + r, 2));

            P = _positions[p] - _objectPos[i];
            V = _velocities[p];
            D = math.length(P);

            if (D > (objR + r))
            {
                continue;
            }

            // need to calculate sr, sd
            float d_sr = ((objCr) * (objCr)) * (Mathf.Pow(objR + r, 2));
            float d_sd = ((objCd) * (objCd)) * (Mathf.Pow(objR + r, 2));
            float numer = D * D; 

            if (numer < d_sr)
            {
                sr = 1 - (numer / d_sr);
            }
            else
            {
                sr = 0;
            }

            if (numer < d_sd)
            {

                sd = 1 - (numer / d_sd);
            }
            else
            {
                //Debug.Log("numer: " + numer + " denom_sd: " + denom_sd);
                continue;
            }

            float repulsive_force;
            float attraction_force;

            if (sr == 0)
            {
                f = P * (sd * objbd * ((math.dot(V, P)) / ((D) * (D))));
                _forces[p] -= f;

                Debug.Log("1) D: " + D + " objB1: " + objb1 + " objB2: " + objb2 + " F: " + f);
            }
            else
            {
                repulsive_force = objb1 / Mathf.Pow((D), m);
                attraction_force = objb2 / Mathf.Pow((D), n);

                f = P * ((sr * (repulsive_force - attraction_force)) - (sd * objbd * ((math.dot(V, P)) / ((D) * (D)))));

                _forces[p] += f;
            }
        }
    }


    List<Transform> GetObjects(Transform parent, bool includeInactive)
    {
        List<Transform> objects = new List<Transform>();
        foreach (Transform child in parent)
        {
            if (child.gameObject.activeInHierarchy || includeInactive)
            {
                objects.Add(child);
            }
            objects.AddRange(GetObjects(child, includeInactive));
        }
        return objects;
    }


    void OnDrawGizmos()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(Vector3.zero, boundsSize);
    }
}




