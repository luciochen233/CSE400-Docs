
module.exports = {
    title: 'CSE400 Robotics',
    description: 'This is a TA-Made Robotics guide/notebook for CSE400 @ Syracuse University',
    markdown: {
        lineNumbers: true
      },
    themeConfig: {
        repo: 'luciochen233/CSE400-Docs',
        docsDir: 'docs/',
        nav: [
            {
                text: 'Main Page',
                link: '/',
            },

            {
                text: 'Extras',
                items: [
                    {
                        text: 'GitHub',
                        items: [
                            {
                                text: 'Repo Page',
                                link: 'https://github.com/luciochen233/CSE400-Docs',
                            },
                        ],
                    },
                    {
                        text: "Lucio's blog",
                        items: [
                            { text: 'Main Page', link: 'https://luciochen.com' }
                        ],
                    },
                ],
            },
        ],
        sidebar: [
            {
                title: 'Getting started',
                collapsable: true,
                children: [
                    "guide/intro",
                    "guide/part2",
                    "guide/part3",
                    "guide/part4",
                    "guide/part5",
                    "guide/part6",
                    "guide/part7"
                ]
            },
            {
                title: 'Programming Laboratory 6',
                collapsable: false,
                children: [
                    "guide/part8",
                    "guide/part9",
                    "guide/part10"
                ]
            },
            {
                title: 'Codes',
                collapsable: false,
                children: [
                    "codes/pythonSimulation",
                    "codes/HardwareInterface",
                    "codes/CustomMessages",
                    "codes/TeensyCodes",
                    "codes/pidController",
                    "codes/LaunchFile"
                ]
            }
              
        ],
        sidebarDepth: 3
    }
}

// themeConfig: {
//     nav: [
//         { text: 'Main Page', link: '/' },
//         {
//             text: 'Quick Guides',
//             items: [
//                 { text: 'Install ubuntu', link: '/ubuntu/' },
//                 { text: 'Install ROS I', link: '/ros/' },
//                 { text: 'Make our first package', link: '/package/' }
//             ]
//         },
//         { text: 'About', link: '/about/' },
//         { text: 'Github', link: 'https://www.github.com/luciochen233' },
//     ],
//     sidebar: {
//         title: 'part 1',
//         collapsable: false,
//         '/ubuntu/': [
//             "",
//             "android1",
//         ],
//         "/ros/": [
//             "",
//             "ios1",
//         ],
//         "/package/": [
//             "pkg1",
//             "pkg2"
//         ]
//     },
//     sidebarDepth: 2,
//     lastUpdated: 'Last Updated',
// },