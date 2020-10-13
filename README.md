#4x4bitFusion project

Overview: Idea is to implement a matrix multiplcation for variable precision inputs (2,4,8 bits only known on memory read). Motivated by  H. Sharma et al., "Bit Fusion: Bit-Level Dynamically Composable Architecture for Accelerating Deep Neural Network," 2018 ACM/IEEE 45th Annual International Symposium on Computer Architecture (ISCA), Los Angeles, CA, 2018, pp. 764-775. My specific implementation is by composing every multiplication into a 4x4 multiplication as opposed to the 2x2 bitBrick structure defined in the paper. 

13th October 2020
Initial Comments: Figured out the hierarchy of the design. Will try to capture it in the diagrams.net SVG file.