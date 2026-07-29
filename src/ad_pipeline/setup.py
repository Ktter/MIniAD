from setuptools import setup
package_name = 'ad_pipeline'
setup(name=package_name, version='0.1.0', packages=[package_name], data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml']),('share/'+package_name+'/launch',['launch/cpu_demo.launch.py','launch/gazebo_demo.launch.py'])], install_requires=['setuptools'], zip_safe=True)
