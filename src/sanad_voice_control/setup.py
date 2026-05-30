from setuptools import find_packages, setup

package_name = "sanad_voice_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            [
                "config/current_lang.yaml",
                "config/system_prompt.yaml",
                "config/telegram_contacts.yaml",
                "config/telegram_messages.yaml",
            ],
        ),
        ("share/" + package_name + "/launch", ["launch/commander.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bod",
    maintainer_email="bodygaber50@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "commander = sanad_voice_control.commander:main",
            "tts_node = sanad_voice_control.tts_node:main",
            "whisper_node = sanad_voice_control.whisper_node:main",
            "telegram_node = sanad_voice_control.telegram_node:main",
            "console_input_node = sanad_voice_control.console_input_node:main",
            "llm_node = sanad_voice_control.llm_node:main",
        ],
    },
)
