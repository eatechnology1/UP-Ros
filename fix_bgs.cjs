const fs = require('fs');
const path = require('path');

function walk(dir) {
    let results = [];
    const list = fs.readdirSync(dir);
    list.forEach(function(file) {
        file = path.join(dir, file);
        const stat = fs.statSync(file);
        if (stat && stat.isDirectory()) { 
            results = results.concat(walk(file));
        } else if (file.endsWith('.vue')) {
            results.push(file);
        }
    });
    return results;
}

const vueFiles = walk('./src/pages').concat(walk('./src/components'));
let changedFiles = 0;

vueFiles.forEach(file => {
    let content = fs.readFileSync(file, 'utf8');
    let original = content;

    // We only want to replace inside <style ...> blocks.
    const styleRegex = /<style[^>]*>([\s\S]*?)<\/style>/g;

    content = content.replace(styleRegex, (match, styleContent) => {
        let newStyle = styleContent
            // Replace background: rgba(15, 23, 42, X)
            .replace(/background:\s*rgba\(\s*15\s*,\s*23\s*,\s*42\s*,\s*[\d.]+\s*\)/g, 'background: var(--bg-surface)')
            // Replace background: #0f172a
            .replace(/background:\s*#0f172a/gi, 'background: var(--bg-surface)')
            // Replace background: #1e1e1e
            .replace(/background:\s*#1e1e1e/gi, 'background: var(--bg-surface)')
            // Replace background: #1e293b
            .replace(/background:\s*#1e293b/gi, 'background: var(--bg-surface-hover)')
            // Replace background: linear-gradient(..., rgba(15,23,42...))
            .replace(/background:\s*linear-gradient\([^;]+(?:rgba\(\s*15\s*,\s*23\s*,\s*42\s*,\s*[\d.]+\s*\)|#0f172a|#1e293b)[^;]*\)/gi, 'background: var(--bg-surface)');
            
        return match.replace(styleContent, newStyle);
    });

    if (content !== original) {
        fs.writeFileSync(file, content);
        changedFiles++;
        console.log("Updated", file);
    }
});

console.log(`Changed ${changedFiles} files.`);
