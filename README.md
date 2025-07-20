Built in Unity 3.7.0 editor version 2022.3.20f1

Build and run provided scenes:

	- open project in Unity, load desired scene, click play

Creating new scene:

	- create a empty game object
	- add the ParticleSim script to the created game object
	- Select a Mesh (i.e. Unity default Sphere mesh) in the ParticleSim Inspector
	- Select a material with GPU instancing enabled (i.e particleMat provided) in the ParticleSim Inspector
	- create a Text - TextMeshPro UI object
	- Select the create TextMeshPro in the ParticleSim Inspector
	- Set Bounds Size to desired simulation window size

Adding a emitter:

	- create a empty game object
	- add to the created game object to the Emitters list in the ParticleSim Inspector

Adding world objects (Only spheres supported):

	- create sphere object
	- nest as a child of the empty game object that has the ParticleSim script linked to it
	
ParticleSim Inspector variables:

	- Mesh: required assignment of a mesh object for the particle
	- Material: required assignment of the material object for the particle (material must have GPU instancing enabled)
	- Canvas Text: UI text - TextMeshPro object for displaying particle count  
	- Num Particles: the number of particles to be generated
	- Particle Mass: mass of a particle
	- Gravity: gravity value
	- Drag C: environmental drag
	- B1: repulsion/attraction
	- Bd: dampening multiplier
	- Cr/Cd: material specifiers
	- Particle Stream: if selected particles will be emitted by the emitters assigned, else particles spawn in a grid formation
	- Emitters: list of emitters to spawn particles from
	- Spawn Rate: rate particles will be emitted from the emitters
	- Initial Velocity: initial velocity of a particle when emitted by a emitter
	- Random Emit: if true emitter will be selected randomly for every particle emit
	- Particle Spacing: spacing of particles when spawned in the grid formation
	- Bounds Size: dimensions of the bounding box for the simulation environment

 
<img width="2000" height="2828" alt="download" src="https://github.com/user-attachments/assets/bdfb8e38-cd8d-4bda-b229-0f5e6405cf92" />
<img width="2000" height="2828" alt="download (1)" src="https://github.com/user-attachments/assets/7eeccbf7-1cae-4e84-b028-207838ea0b6d" />
<img width="2000" height="2828" alt="download (2)" src="https://github.com/user-attachments/assets/9f6fce27-4beb-4613-b6e2-9b3adc07f4e1" />
<img width="2000" height="2828" alt="download (3)" src="https://github.com/user-attachments/assets/fadee101-eef1-4948-bee6-bb441179ee33" />
