
import xml.etree.ElementTree as ET
import os

def verify_xml_and_assets():
    """Verify the XML file structure and check for required assets"""

    xml_file = 'ur5e_with_mop_and_dust_fixed.xml'

    print("🔍 XML and Asset Verification")
    print("=" * 40)

    # Check XML file exists
    if not os.path.exists(xml_file):
        print(f"❌ XML file '{xml_file}' not found!")
        return False

    try:
        # Parse XML
        tree = ET.parse(xml_file)
        root = tree.getroot()

        print(f"✅ XML file loaded: {xml_file}")
        print(f"📄 Root tag: {root.tag}")
        print(f"🏷️  Model name: {root.get('model')}")

        # Check dust particles
        worldbody = root.find('worldbody')
        dust_bodies = [body for body in worldbody.findall('body') 
                      if body.get('name') and body.get('name').startswith('dust_particle_')]

        print(f"✨ Number of dust particles: {len(dust_bodies)}")

        # Check for dust material
        asset = root.find('asset')
        dust_material = asset.find("material[@name='dust_material']")
        dust_mesh = asset.find("mesh[@name='dust_ball_mesh']")

        if dust_material is not None:
            print("✅ Dust material found")
        else:
            print("❌ Dust material missing")

        if dust_mesh is not None:
            print("✅ Dust mesh definition found")
            mesh_file = dust_mesh.get('file')
            print(f"🔘 Dust mesh file: {mesh_file}")

            # Check if STL file exists
            stl_path = os.path.join('assets', mesh_file)
            if os.path.exists(stl_path):
                print(f"✅ STL file found: {stl_path}")
            else:
                print(f"❌ STL file missing: {stl_path}")
                return False
        else:
            print("❌ Dust mesh definition missing")
            return False

        # Check actuators
        actuators = root.find('actuator')
        if actuators is not None:
            actuator_list = actuators.findall('position')
            print(f"🦾 Number of actuators: {len(actuator_list)}")

        print("\n✅ XML verification completed successfully!")
        return True

    except Exception as e:
        print(f"❌ XML verification failed: {e}")
        return False

if __name__ == "__main__":
    verify_xml_and_assets()
