from setuptools import setup
package_name='ad_tools'
setup(name=package_name,version='0.1.0',packages=[package_name],data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml'])],install_requires=['setuptools'],entry_points={'console_scripts':['evaluate_run=ad_tools.evaluate_run:main','replay_topics=ad_tools.replay_topics:main']})
