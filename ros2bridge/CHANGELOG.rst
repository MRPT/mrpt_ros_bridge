^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_libros_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2026-02-06)
------------------
* Merge pull request `#1 <https://github.com/MRPT/mrpt_ros_bridge/issues/1>`_ from MRPT/feat/export-pointcloud-color
  Fix: export RGB color clouds in the expected format by RViz/PCL
* Fix: export RGB color clouds in the expected format by RViz/PCL
* Support LaserScan.scan_time for mrpt 2.15.6
* Contributors: Jose Luis Blanco-Claraco

3.1.1 (2025-12-26)
------------------
* Prepare for API change in mrpt 2.15.4
* Contributors: Jose Luis Blanco-Claraco

3.1.0 (2025-12-23)
------------------
* PointCloud2 to CGenericPointsMap: parse remaining fields of unsupported types as float
* Contributors: Jose Luis Blanco-Claraco

3.0.2 (2025-11-08)
------------------
* FIX: Wrong find_dependency() on mrpt-apps, while it should be now mrpt-maps for minimal dependencies
* Contributors: Jose Luis Blanco-Claraco

3.0.1 (2025-11-07)
------------------
* Fix package.xml so packages downstream install the required dependencies
* Finer grained dependencies on the part of mrpt that is really used
* Contributors: Jose Luis Blanco-Claraco

3.0.0 (2025-11-07)
------------------
* Release as independent repository. Moved out from the MRPT/mrpt repository.


2.14.15 (2024-10-12)
----------------------
