using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.UrdfImporter;

public static class DaVinciImportTool
{
    [MenuItem("Tools/da Vinci/Import PSM1 Demo Scene")]
    public static void ImportPsm1DemoScene()
    {
        const string urdfPath = "Assets/DaVinciPSM/PSM1.urdf";
        AssetDatabase.Refresh();
        DaVinciMaterialFixer.ConvertMaterialsToUrp();

        EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);

        var settings = new ImportSettings
        {
            chosenAxis = ImportSettings.axisType.zAxis,
            convexMethod = ImportSettings.convexDecomposer.unity
        };

        var enumerator = UrdfRobotExtensions.Create(urdfPath, settings, false);
        GameObject robot = null;
        while (enumerator.MoveNext())
        {
            robot = enumerator.Current;
        }

        if (robot == null)
        {
            Debug.LogError("DAVINCI_IMPORT_FAILED: robot GameObject is null. Check preceding Console errors.");
            return;
        }

        robot.transform.position = Vector3.zero;

        // The importer leaves the root ArticulationBody movable by default, so the whole chain
        // free-falls under gravity in Play mode unless it's pinned in place here.
        var rootBody = robot.GetComponentsInChildren<ArticulationBody>().FirstOrDefault(ab => ab.isRoot);
        if (rootBody != null)
        {
            rootBody.immovable = true;
        }
        else
        {
            Debug.LogWarning("DAVINCI_IMPORT_WARN: no root ArticulationBody found to pin in place; robot may fall under gravity.");
        }

        if (robot.GetComponent<DaVinciWasdController>() == null)
        {
            robot.AddComponent<DaVinciWasdController>();
        }

        var camGo = GameObject.Find("Main Camera");
        if (camGo != null)
        {
            camGo.transform.position = new Vector3(1.2f, 0.9f, 1.2f);
            camGo.transform.LookAt(robot.transform.position + Vector3.up * 0.3f);
        }

        if (Object.FindFirstObjectByType<Light>() == null)
        {
            var lightGo = new GameObject("Directional Light");
            var light = lightGo.AddComponent<Light>();
            light.type = LightType.Directional;
            lightGo.transform.rotation = Quaternion.Euler(50, -30, 0);
        }

        var scene = SceneManager.GetActiveScene();
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene, "Assets/Scenes/DaVinciDemo.unity");

        Debug.Log("DAVINCI_IMPORT_OK robot=" + robot.name + " links=" + settings.linksLoaded);
    }
}
