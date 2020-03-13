using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using System.IO;
using UnityEngine;
using System.Reflection;

namespace TagSlamTracker
{
    public class TagSlamTracker
    {
        // FUNCTION IMPORTED FROM DLL:
        [DllImport("ViSPUnity", CallingConvention = CallingConvention.Cdecl, EntryPoint = "PoseFromAprilTag")]
        private static extern void PoseFromAprilTag(byte[] bitmap, int height, int width,
            double cam_px, double cam_py, double cam_u0, double cam_v0, double cam_kud, double cam_kdu, double tagSize,
            double[] coord, double[] U, double[] V, double[] W, double[] T, double[] h, double[] w, double[] apr, int[] tag_id, int[] is_tag_detected);

        [DllImport("ViSPUnity", CallingConvention = CallingConvention.Cdecl, EntryPoint = "PoseFromAprilTagWithSlam")]
        private static extern void PoseFromAprilTagWithSlam(byte[] bitmap, int height, int width,
            double cam_px, double cam_py, double cam_u0, double cam_v0, double cam_kud, double cam_kdu, double tagSize, double distance_to_tag_to_reinit,
            double[] coord, double[] U, double[] V, double[] W, double[] T, double[] h,
            double[] w, double[] apr, int[] tag_id, int[] is_tag_detected, double[] distance, int[] Is_slam_tracking);

        [DllImport("ViSPUnity", CallingConvention = CallingConvention.Cdecl, EntryPoint = "Shutdown")]
        private static extern void Shutdown_VispUnity();

        [DllImport("ViSPUnity", CallingConvention = CallingConvention.Cdecl, EntryPoint = "Init")]
        private static extern void Init_VispUnity(string path_to_slam_vocab, string path_to_camera_params, int bitmap_height, int bitmap_width);


        private static int _height;
        private static int _width;
        private static double _tag_size;
        private static bool _shutdown = true;
        private static string _path_to_camera_params;
        private static string _path_to_slam_vocab;
        private static Camera _camera;

        public static void Init(int height, int width, double tagSize, string path_to_camera_params, string path_to_slam_vocab)
        {
            if (_shutdown)
            {
                _height = height;
                _width = width;
                _tag_size = tagSize;
                _path_to_camera_params = path_to_camera_params;
                _path_to_slam_vocab = path_to_slam_vocab;
                
                var input = new StreamReader(path_to_camera_params);
                var yaml = input.ReadToEnd();
                _camera = new Camera();

                _camera.Init_From_YAML(yaml);

                Init_VispUnity(_path_to_slam_vocab, _path_to_camera_params, _height, _width);
                _shutdown = false;
            }
            else
            {
                throw new System.InvalidOperationException("Must shutdown before reinitialisation");
            }
        }

        public static void Shutdown()
        {
            if (!_shutdown)
            {
                Shutdown_VispUnity();
                _shutdown = true;
            }
            else
            {
                throw new System.InvalidOperationException("Must initialise before shutdown");
            }
        }


        public static void GetPose(Color32[] image, double distance_to_tag_to_reinit, out bool is_slam_tracking, out bool is_tag_detected, out int tag_id, out double tag_distance, out Quaternion rotation, out Vector3 translation, out Vector3 scale)
        {
            if (!_shutdown)
            {
                 
                byte[] ba = Color32ArrayToByteArray(image);

                int[] are_tags_detected_returned = new int[1];
                int[] is_slam_tracking_returned = new int[1];

                double[] U = new double[4];
                double[] V = new double[4];
                double[] W = new double[4];
                double[] T = new double[4];

                double[] distance = new double[1];
                string[] info = new string[3];
                double[] coord = new double[4];

                int[] tag_ids = new int[1];
                double[] h = new double[1];
                double[] w = new double[1];
                double[] apr = new double[6];

                    
                PoseFromAprilTagWithSlam(ba, _height, _width, _camera.fx, _camera.fx, _camera.cx, _camera.cy, _camera.k1, -_camera.k1,
                        _tag_size, distance_to_tag_to_reinit, coord, U, V, W, T, h, w, apr, tag_ids, are_tags_detected_returned, distance, is_slam_tracking_returned);
                   

                //PoseFromAprilTag(ba, wct.height, wct.width, cam_px, cam_py, cam_u0, cam_v0, cam_kud, cam_kdu,
                //tagSize, coord, U, V, W, T, h, w, apr, tag_id, Is_tag_detected);

                tag_id = tag_ids[0];
                tag_distance = distance[0];

                is_tag_detected = (are_tags_detected_returned[0] == 1);
                is_slam_tracking = (is_slam_tracking_returned[0] == 1);

                if ((is_tag_detected || is_slam_tracking)
                    && (Math.Abs(T[0] * T[1] * T[2]) > 0.0000001))
                {
                    Matrix4x4 transformationM = new Matrix4x4(); // from OpenCV
                    transformationM.SetRow(0, new Vector4((float)U[0], (float)V[0], (float)W[0], (float)T[0]));
                    transformationM.SetRow(1, new Vector4((float)U[1], (float)V[1], (float)W[1], (float)T[1]));
                    transformationM.SetRow(2, new Vector4((float)U[2], (float)V[2], (float)W[2], (float)T[2]));
                    transformationM.SetRow(3, new Vector4(0, 0, 0, 1));

                    Matrix4x4 invertZM = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(1, 1, -1));
                    Matrix4x4 invertYM = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(1, -1, 1));

                    // right-handed coordinates system (OpenCV) to left-handed one (Unity)
                    Matrix4x4 ARM = invertYM * transformationM;

                    // Apply Z-axis inverted matrix.
                    ARM = ARM * invertZM;

                    Matrix4x4 inv_ARM = ARM.inverse;
                    rotation = ExtractRotationFromMatrix(ref inv_ARM);
                    translation = ExtractTranslationFromMatrix(ref inv_ARM);
                    scale = ExtractScaleFromMatrix(ref inv_ARM);
                    return;
                }
                else
                {
                    rotation = Quaternion.identity;
                    translation = Vector3.zero;
                    scale = new Vector3(1f, 1f, 1f);
                    return;
                }
                
            }
            else
            {
                throw new System.InvalidOperationException("Initialise before getting pose");
            }
        }

        /// <summary>
        /// Extract translation from transform matrix.
        /// </summary>
        /// <param name="matrix">Transform matrix. This parameter is passed by reference
        /// to improve performance; no changes will be made to it.</param>
        /// <returns>
        /// Translation offset.
        /// </returns>
        public static Vector3 ExtractTranslationFromMatrix(ref Matrix4x4 matrix)
        {
            Vector3 translate;
            translate.x = matrix.m03;
            translate.y = matrix.m13;
            translate.z = matrix.m23;
            return translate;
        }

        /// <summary>
        /// Extract rotation quaternion from transform matrix.
        /// </summary>
        /// <param name="matrix">Transform matrix. This parameter is passed by reference
        /// to improve performance; no changes will be made to it.</param>
        /// <returns>
        /// Quaternion representation of rotation transform.
        /// </returns>
        public static Quaternion ExtractRotationFromMatrix(ref Matrix4x4 matrix)
        {
            Vector3 forward;
            forward.x = matrix.m02;
            forward.y = matrix.m12;
            forward.z = matrix.m22;

            Vector3 upwards;
            upwards.x = matrix.m01;
            upwards.y = matrix.m11;
            upwards.z = matrix.m21;

            return Quaternion.LookRotation(forward, upwards);
        }

        /// <summary>
        /// Extract scale from transform matrix.
        /// </summary>
        /// <param name="matrix">Transform matrix. This parameter is passed by reference
        /// to improve performance; no changes will be made to it.</param>
        /// <returns>
        /// Scale vector.
        /// </returns>
        public static Vector3 ExtractScaleFromMatrix(ref Matrix4x4 matrix)
        {
            Vector3 scale;
            scale.x = new Vector4(matrix.m00, matrix.m10, matrix.m20, matrix.m30).magnitude;
            scale.y = new Vector4(matrix.m01, matrix.m11, matrix.m21, matrix.m31).magnitude;
            scale.z = new Vector4(matrix.m02, matrix.m12, matrix.m22, matrix.m32).magnitude;
            return scale;
        }

        /// <summary>
        /// Extract position, rotation and scale from TRS matrix.
        /// </summary>
        /// <param name="matrix">Transform matrix. This parameter is passed by reference
        /// to improve performance; no changes will be made to it.</param>
        /// <param name="localPosition">Output position.</param>
        /// <param name="localRotation">Output rotation.</param>
        /// <param name="localScale">Output scale.</param>
        public static void DecomposeMatrix(ref Matrix4x4 matrix, out Vector3 localPosition, out Quaternion localRotation, out Vector3 localScale)
        {
            localPosition = ExtractTranslationFromMatrix(ref matrix);
            localRotation = ExtractRotationFromMatrix(ref matrix);
            localScale = ExtractScaleFromMatrix(ref matrix);
        }

        /// <summary>
        /// Set transform component from TRS matrix.
        /// </summary>
        /// <param name="transform">Transform component.</param>
        /// <param name="matrix">Transform matrix. This parameter is passed by reference
        /// to improve performance; no changes will be made to it.</param>
        public static void SetTransformFromMatrix(Transform transform, ref Matrix4x4 matrix)
        {
            transform.localPosition = ExtractTranslationFromMatrix(ref matrix);
            transform.localRotation = ExtractRotationFromMatrix(ref matrix);
            transform.localScale = ExtractScaleFromMatrix(ref matrix);
        }

        //Function for converting into Byte Array to be sent to functions in DLL
        private static byte[] Color32ArrayToByteArray(Color32[] colors)
        {
            if (colors == null || colors.Length == 0)
                return null;

            int length = colors.Length;
            byte[] bytes = new byte[length];
            int value = 0;

            for (int i = 0; i < colors.Length; i++)
            {
                value = (colors[i].r + colors[i].g + colors[i].b) / 3;
                bytes[colors.Length - i - 1] = (byte)value;
            }

            return bytes;
        }

    }

    class Camera
    {
        private double _fx;
        private double _fy;
        private double _cx;
        private double _cy;
        private double _k1;
        private double _k2;
        private double _p1;
        private double _p2;
        private double _fps;
        
        public double fx { get => _fx; set => _fx = value; }
        public double fy { get => _fy; set => _fy = value; }
        public double cx { get => _cx; set => _cx = value; }
        public double cy { get => _cy; set => _cy = value; }
        public double k1 { get => _k1; set => _k1 = value; }
        public double k2 { get => _k2; set => _k2 = value; }
        public double p1 { get => _p1; set => _p1 = value; }
        public double p2 { get => _p2; set => _p2 = value; }
        public double fps { get => _fps; set => _fps = value; }
        
        public void Init_From_YAML(string yaml)
        {
            // method horrible but couldn't parse with YAMLDotNet
            
            StringReader reader = new StringReader(yaml);
            Type camType = typeof(Camera);

            while (true)
            {
                string line = reader.ReadLine();
                if (line != null)
                {
                   if (line.StartsWith("Camera"))
                    {
                        string full_key = line.Split(':')[0];
                        string key = full_key.Split('.')[1];
                        double value = Double.Parse(line.Split(':')[1]);

                        try
                        {
                            PropertyInfo pi = camType.GetProperty(key);
                            pi.SetValue(this, value);
                        }
                        catch (System.NullReferenceException)
                        {
                            Debug.Log("No property found for " + key);
                        }
                    }
                }
                else
                {
                    break;
                }
            }

        }
    }
}

