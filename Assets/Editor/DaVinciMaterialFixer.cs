using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public static class DaVinciMaterialFixer
{
    private const string RootFolder = "Assets/DaVinciPSM";

    [MenuItem("Tools/da Vinci/Fix Pink Materials (Convert to URP)")]
    public static void ConvertMaterialsToUrpMenuItem()
    {
        ConvertMaterialsToUrp();
    }

    // Returns how many materials were converted, so callers (e.g. the import tool) can fold this in.
    public static int ConvertMaterialsToUrp()
    {
        Shader urpLit = Shader.Find("Universal Render Pipeline/Lit");
        if (urpLit == null)
        {
            Debug.LogError("DAVINCI_FIX_FAILED: 'Universal Render Pipeline/Lit' shader not found. " +
                "Confirm URP is installed and a URP asset is assigned in Project Settings > Graphics.");
            return 0;
        }

        var materials = new HashSet<Material>();

        foreach (var guid in AssetDatabase.FindAssets("t:Material", new[] { RootFolder }))
        {
            var mat = AssetDatabase.LoadAssetAtPath<Material>(AssetDatabase.GUIDToAssetPath(guid));
            if (mat != null) materials.Add(mat);
        }

        foreach (var guid in AssetDatabase.FindAssets("t:Model", new[] { RootFolder }))
        {
            string path = AssetDatabase.GUIDToAssetPath(guid);
            foreach (var obj in AssetDatabase.LoadAllAssetsAtPath(path))
            {
                if (obj is Material mat) materials.Add(mat);
            }
        }

        int converted = 0;
        foreach (var mat in materials)
        {
            if (mat.shader == urpLit) continue;

            Color baseColor = Color.white;
            if (mat.HasProperty("_Color")) baseColor = mat.GetColor("_Color");
            else if (mat.HasProperty("_BaseColor")) baseColor = mat.GetColor("_BaseColor");

            Texture mainTex = mat.HasProperty("_MainTex") ? mat.GetTexture("_MainTex") : null;

            mat.shader = urpLit;
            mat.SetColor("_BaseColor", baseColor);
            if (mainTex != null) mat.SetTexture("_BaseMap", mainTex);

            // The dvrk_model meshes carry flat COLLADA/OBJ colors with no UV image textures at all,
            // so there's nothing to "apply a texture" from. This gives them a brushed-steel look
            // instead of the flat/matte default, which is closer to the real instrument finish.
            mat.SetFloat("_Metallic", 0.75f);
            mat.SetFloat("_Smoothness", 0.55f);

            EditorUtility.SetDirty(mat);
            converted++;
        }

        AssetDatabase.SaveAssets();
        Debug.Log($"DAVINCI_FIX_OK converted={converted} materials to URP/Lit under {RootFolder}");
        return converted;
    }
}
