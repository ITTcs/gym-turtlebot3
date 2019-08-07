import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="gym_turtlebot3",
    version="0.0.1",
    author="Eduardo Avelar",
    author_email="eavelardev@gmail.com",
    description="Gym env for TurtleBot3 robot",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ITTcs/gym-turtlebot3",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    install_requires=['gym']
)
